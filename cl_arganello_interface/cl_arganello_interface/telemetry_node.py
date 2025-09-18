#!/usr/bin/env python3
import json
import math
import time
import threading
import serial
import csv
from pathlib import Path
from typing import Optional, List, Dict
from collections import deque
from typing import Union
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cl_arganello_interface.msg import RopeCommand
from cl_arganello_interface.msg import DebugMessage


class TelemetryNode(Node):
    """
    Reads CSV telemetry from MCU, republishes raw lines, computes robust velocities:
      - fixed dt from poll_rate_hz (no reliance on CSV time for derivatives)
      - unwrap motor (0..1) → rev, roller (0..CPR-1) → counts
      - LPF on unwrapped positions (NOT on velocity)
      - windowed derivative across k samples
      - outlier hold, brake gating, rope-speed sanity clamp
    """

    def __init__(self):
        super().__init__("telemetry_node")

        # ── Parameters ─────────────────────────────────────────────────────────
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("poll_rate_hz", 200.0)

        self.declare_parameter("side", "left")  # "left" | "right"
        self.declare_parameter(
            "config_path",
            str(Path(__file__).resolve().parent.parent / "config" / "arganelloTelemetry.json"),
        )
        self.declare_parameter("csv_expect_header", True)
        self.declare_parameter("replace_first_column_with_unix_time", False)
        self.declare_parameter("debug_mode", False)

        # Sync behavior
        self.declare_parameter("send_sync_on_start", True)
        self.declare_parameter("resync_interval_s", 0.0)  # 0 => disabled
        self.declare_parameter("sync_epoch_unit", "ms")   # "ms" (recommended) or "ns"
        self.declare_parameter("append_pc_time_ns", False)

        # Geometry/config (override at launch if needed)
        self.declare_parameter("sync_roller_radius_m", 0.0025)  # meters (← set your real radius)
        self.declare_parameter("sync_roller_cpr", 2400)

        # Limits & filters (override at launch if needed)
        self.declare_parameter("max_motor_rps", 20.0)
        self.declare_parameter("max_roller_rps", 20.0)
        self.declare_parameter("count_deadband", 2)
        self.declare_parameter("lpf_fc_pos_motor", 5.0)     # Hz, on motor position
        self.declare_parameter("lpf_fc_pos_roller", 5.0)    # Hz, on roller counts
        self.declare_parameter("brake_zero_rps", 0.2)       # ~12 rpm threshold
        self.declare_parameter("phys_max_rope_m_s", 5.0)

        # ── Read params ────────────────────────────────────────────────────────
        self.port = self.get_parameter("serial_port").value
        self.baud = int(self.get_parameter("baud").value)
        self.poll_rate_hz = float(self.get_parameter("poll_rate_hz").value)

        side_in = str(self.get_parameter("side").value).strip().lower()
        self.side = side_in if side_in in ("left", "right") else "left"

        self.config_path = self.get_parameter("config_path").value
        self.expect_header = bool(self.get_parameter("csv_expect_header").value)
        self.replace_col0_with_unix = bool(self.get_parameter("replace_first_column_with_unix_time").value)
        self.debug_mode = bool(self.get_parameter("debug_mode").value)

        self.send_sync_on_start = bool(self.get_parameter("send_sync_on_start").value)
        self.resync_interval_s = float(self.get_parameter("resync_interval_s").value)
        self.sync_epoch_unit = str(self.get_parameter("sync_epoch_unit").value).strip().lower()
        self.append_pc_time_ns = bool(self.get_parameter("append_pc_time_ns").value)

        self.sync_roller_radius_m = float(self.get_parameter("sync_roller_radius_m").value)
        self.sync_roller_cpr = int(self.get_parameter("sync_roller_cpr").value)

        self.max_motor_rps = float(self.get_parameter("max_motor_rps").value)
        self.max_roller_rps = float(self.get_parameter("max_roller_rps").value)
        self.count_deadband = int(self.get_parameter("count_deadband").value)
        self.lpf_fc_pos_motor = float(self.get_parameter("lpf_fc_pos_motor").value)
        self.lpf_fc_pos_roller = float(self.get_parameter("lpf_fc_pos_roller").value)
        self.brake_zero_rps = float(self.get_parameter("brake_zero_rps").value)
        self.phys_max_rope_m_s = float(self.get_parameter("phys_max_rope_m_s").value)

        # ── Serial ─────────────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.005)
            self.get_logger().info(f"✅ Serial opened: {self.port} @ {self.baud}")
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().fatal(f"❌ Failed to open serial: {e}")
            raise

        # --- Sticky “last valid” holders (init before threads start)
        self._last_valid_motor_pos_norm: float | None = None


        # ── Pubs/Subs/Services (namespaced by side) ───────────────────────────
        base = f"/winch/{self.side}"
        self.pub_csv = self.create_publisher(String, f"{base}/telemetry/csv", 10)
        self.pub_debug = self.create_publisher(DebugMessage, f"{base}/telemetry/debug", 10)

        self.create_subscription(String, f"{base}/set_motor_mode", self._set_motor_mode_cb, 10)
        self.create_subscription(RopeCommand, f"{base}/command", self._rope_command_cb, 10)

        self.create_service(Trigger, f"{base}/brake_engage", self._srv_brake_engage)
        self.create_service(Trigger, f"{base}/brake_disengage", self._srv_brake_disengage)
        self.create_service(Trigger, f"{base}/sync_now", self._srv_sync_now)

        # ── TX queue & CSV header tracking ────────────────────────────────────
        self.tx_queue: deque[str] = deque()
        self.header: List[str] = []
        self.name_to_idx: Dict[str, int] = {}
        self._stop = False

        # ── State for processing ───────────────────────────────────────────────
        # Unwrapped positions (pre-filter)
        self._motor_unwrapped_rev: Optional[float] = None
        self._roller_unwrapped_counts: Optional[float] = None

        # Filtered positions history (for windowed derivative)
        self._hist_pos_motor_filt: deque = deque(maxlen=8)
        self._hist_pos_roller_filt: deque = deque(maxlen=8)

        # Speeds (rev/s)
        self.motor_speed: float = 0.0
        self.sync_roller_speed: float = 0.0

        # ── SYNC & CONFIG ─────────────────────────────────────────────────────
        if self.send_sync_on_start:
            self._send_sync()
        if self.resync_interval_s > 0:
            self.create_timer(self.resync_interval_s, self._send_sync)

        cfg = self._load_config(self.config_path)
        if cfg:
            self.send_cmd("CONFIG " + json.dumps(cfg, separators=(",", ":")))
        else:
            self.get_logger().warn("No CONFIG JSON found; device may not stream.")

        # ── Start IO threads ───────────────────────────────────────────────────
        threading.Thread(target=self._serial_reader_loop, name="serial-reader", daemon=True).start()
        threading.Thread(target=self._serial_writer_loop, name="serial-writer", daemon=True).start()

        if self.debug_mode:
            threading.Thread(target=self._stdin_loop, name="stdin", daemon=True).start()

    # ── Public: enqueue a raw line to MCU ──────────────────────────────────────
    def send_cmd(self, line: str) -> None:
        if line:
            self.tx_queue.append(line if line.endswith("\n") else (line + "\n"))

    # ── Brake services ────────────────────────────────────────────────────────
    def _srv_brake_engage(self, req, res):
        cmd = "set_brake 1"
        self.get_logger().info(f"→ {cmd}")
        self.send_cmd(cmd)
        res.success = True
        res.message = "sent: set_brake 1"
        return res

    def _srv_brake_disengage(self, req, res):
        cmd = "set_brake 0"
        self.get_logger().info(f"→ {cmd}")
        self.send_cmd(cmd)
        res.success = True
        res.message = "sent: set_brake 0"
        return res

    # New: manual sync service
    def _srv_sync_now(self, req, res):
        epoch = self._send_sync()
        res.success = True
        res.message = f"sent: sync {epoch}"
        return res

    # ── set_motor_mode callback ───────────────────────────────────────────────
    def _set_motor_mode_cb(self, msg: String) -> None:
        m = (msg.data or "").strip().lower()
        if m == "idle":
            self.send_cmd("send_odrive w axis0.requested_state 1")
        elif m == "closed_loop_torque":
            self.send_cmd("send_odrive w axis0.controller.config.input_mode 1")
            self.send_cmd("send_odrive w axis0.controller.config.control_mode 1")
            self.send_cmd("send_odrive w axis0.requested_state 8")
        elif m == "closed_loop_velocity":
            self.send_cmd("send_odrive w axis0.controller.config.input_mode 1")
            self.send_cmd("send_odrive w axis0.controller.config.control_mode 2")
            self.send_cmd("send_odrive w axis0.requested_state 8")
        elif m == "closed_loop_position":
            self.send_cmd("send_odrive w axis0.controller.config.input_mode 1")
            self.send_cmd("send_odrive w axis0.controller.config.control_mode 3")
            self.send_cmd("send_odrive w axis0.requested_state 8")
        else:
            self.get_logger().warn(
                "Unknown motor mode. Use: idle, closed_loop_torque, closed_loop_velocity, closed_loop_position"
            )

    # ── rope_command_callback ─────────────────────────────────────────────────
    def _rope_command_cb(self, msg: RopeCommand) -> None:
        self.get_logger().info(
            f"[command] force={msg.rope_force:.3f}, vel={msg.rope_velocity:.3f}, pos={msg.rope_position:.3f}"
        )

    # ── Reader/Writer loops ───────────────────────────────────────────────────
    def _serial_reader_loop(self) -> None:
        """Decode lines from serial and hand them to process_csv(line)."""
        while not self._stop:
            try:
                if not self.ser.in_waiting:
                    time.sleep(0.0005)
                    continue
                raw = self.ser.readline()  # up to '\n' or timeout
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
                if not line:
                    continue
                self.process_csv(line)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                time.sleep(0.01)

    def _serial_writer_loop(self) -> None:
        """Sends queued commands in FIFO bursts when RX is idle."""
        while not self._stop:
            try:
                if self.tx_queue and not self.ser.in_waiting:
                    for _ in range(4):
                        if not self.tx_queue or self.ser.in_waiting:
                            break
                        out = self.tx_queue.popleft()
                        self.ser.write(out.encode("utf-8"))
                else:
                    time.sleep(0.0005)
            except Exception as e:
                self.get_logger().warn(f"Serial write error: {e}")
                time.sleep(0.01)

    # ── Process one CSV line ──────────────────────────────────────────────────
    def process_csv(self, line: str) -> None:
        """
        Fixed-dt (from poll_rate_hz), LPF on positions, windowed derivative.
        Publishes DebugMessage.
        """
        # 0) republish raw CSV
        self.pub_csv.publish(String(data=line))

        # 1) parse row & header
        try:
            row = next(csv.reader([line]))
        except Exception:
            return
        if not row:
            return

        if self.expect_header and not self.header:
            self.header = [h.strip() for h in row]
            self.name_to_idx = {n: i for i, n in enumerate(self.header)}
            return
        if self.expect_header and not self.header:
            return

        def idx(name: str, fallback: int):
            return self.name_to_idx.get(name, fallback)

        def cell(name: str, pos: int):
            i = idx(name, pos)
            return row[i].strip() if 0 <= i < len(row) else None

        # 2) timestamp (kept only for logging; derivatives use fixed dt)
        t_s = None
        s = cell("epoch_ms", 0)
        if s not in (None, ""):
            try:
                t_s = float(s) / 1_000.0
            except Exception:
                t_s = None
        if t_s is None:
            s = cell("micros", 0)
            if s not in (None, ""):
                try:
                    t_s = float(s) / 1_000_000.0
                except Exception:
                    t_s = None
        if t_s is None:
            t_s = time.time()

        # 3) raw fields
        brake_status = (cell("brake", 1) in ("1", "true", "True", "TRUE"))

        s = cell("ibus", 2)
        try:
            current = float(s) if s not in (None, "") else None
        except Exception:
            current = None

        s = cell("motor_torque", 3)
        try:
            motor_torque = float(s) if s not in (None, "") else None
        except Exception:
            motor_torque = None

        s = cell("syncronous_roller_raw_wrapped", 4)
        try:
            sync_raw = int(float(s)) if s not in (None, "") else None
        except Exception:
            sync_raw = None

        # Ensure the holder exists even if __init__ changes order
        if not hasattr(self, "_last_valid_sync_raw"):
            self._last_valid_sync_raw = None

        # HOLD last valid if current is 0 or None
        sr = sync_raw
        if sr is None or sr == 0:
            if self._last_valid_sync_raw is not None:
                sr = self._last_valid_sync_raw
        else:
            self._last_valid_sync_raw = sr

        sync_raw = sr
        sync_raw = self.glitch_outlier_filter(sync_raw,
                                      name="sync_raw",
                                      window=15,
                                      k=3.5,
                                      max_step=self.max_roller_rps * self.sync_roller_cpr / self.poll_rate_hz,
                                      persist=3)



        # motor_raw_wrapped: normalized [0,1)
        s = cell("motor_raw_wrapped", 5)
        try:
            motor_pos_norm = float(s) if s not in (None, "") else None
        except Exception:
            motor_pos_norm = None

        # Ensure the holder exists even if __init__ ever changes order
        if not hasattr(self, "_last_valid_motor_pos_norm"):
            self._last_valid_motor_pos_norm = None

        # HOLD last valid if current is 0.0 or None
        mp = motor_pos_norm
        if mp is None or mp == 0.0:
            if self._last_valid_motor_pos_norm is not None:
                mp = self._last_valid_motor_pos_norm
        else:
            self._last_valid_motor_pos_norm = mp

        motor_pos_norm = mp
        motor_pos_norm = self.moving_average_filter(motor_pos_norm,window=25,name="motor_pos")


        # 4) constants for math
        poll_rate_hz = self.poll_rate_hz
        fixed_dt = 1.0 / max(1e-6, poll_rate_hz)

        cpr_i = int(self.sync_roller_cpr)     # modulo uses int
        cpr = float(cpr_i)                    # math uses float
        radius_m = float(self.sync_roller_radius_m)

        max_motor_rps = self.max_motor_rps
        max_roller_rps = self.max_roller_rps
        count_deadband = self.count_deadband
        lpf_fc_pos_motor = self.lpf_fc_pos_motor
        lpf_fc_pos_roller = self.lpf_fc_pos_roller
        brake_zero_rps = self.brake_zero_rps
        phys_max_rope_m_s = self.phys_max_rope_m_s
        tau2pi = 2.0 * math.pi

        # helper: 1st-order LPF on positions
        def lpf_pos(prev_val: Optional[float], x: Optional[float], dt: float, fc: float) -> Optional[float]:
            if x is None:
                return prev_val
            if prev_val is None or fc <= 0 or dt <= 0:
                return x if prev_val is None else prev_val
            tau = 1.0 / (2.0 * math.pi * fc)
            a = dt / (tau + dt)
            return prev_val + a * (x - prev_val)

        # 5) unwrap motor (0..1) → continuous rev
        motor_unwrapped_rev = self._motor_unwrapped_rev
        if motor_pos_norm is not None:
            if motor_unwrapped_rev is None:
                motor_unwrapped_rev = float(motor_pos_norm)
            else:
                frac_prev = motor_unwrapped_rev - math.floor(motor_unwrapped_rev)
                dm = motor_pos_norm - frac_prev
                if dm > 0.5:
                    dm -= 1.0
                if dm < -0.5:
                    dm += 1.0
                motor_unwrapped_rev = motor_unwrapped_rev + dm
        if motor_unwrapped_rev is None:
            motor_unwrapped_rev = self._motor_unwrapped_rev
        self._motor_unwrapped_rev = motor_unwrapped_rev

        # 6) unwrap roller (0..CPR-1) → continuous counts
        roller_unwrapped_counts = self._roller_unwrapped_counts
        if sync_raw is not None:
            if roller_unwrapped_counts is None:
                roller_unwrapped_counts = float(sync_raw)
            else:
                prev_wrap = int(round(roller_unwrapped_counts)) % cpr_i
                dr = int(sync_raw) - prev_wrap
                half = cpr / 2.0
                if dr > half:
                    dr -= cpr_i
                elif dr < -half:
                    dr += cpr_i
                if abs(dr) < count_deadband:
                    dr = 0
                roller_unwrapped_counts = roller_unwrapped_counts + float(dr)
        if roller_unwrapped_counts is None:
            roller_unwrapped_counts = self._roller_unwrapped_counts
        self._roller_unwrapped_counts = roller_unwrapped_counts

        # 7) LPF on positions (NOT on velocity)
        if not hasattr(self, "_pos_motor_filt"):
            self._pos_motor_filt = motor_unwrapped_rev
        else:
            self._pos_motor_filt = lpf_pos(self._pos_motor_filt, motor_unwrapped_rev, fixed_dt, lpf_fc_pos_motor)

        if not hasattr(self, "_pos_roller_filt"):
            self._pos_roller_filt = roller_unwrapped_counts
        else:
            self._pos_roller_filt = lpf_pos(self._pos_roller_filt, roller_unwrapped_counts, fixed_dt, lpf_fc_pos_roller)

        # push to history for windowed derivative
        self._hist_pos_motor_filt.append(self._pos_motor_filt)
        self._hist_pos_roller_filt.append(self._pos_roller_filt)

        # 8) windowed derivative using fixed dt (with outlier HOLD)
        motor_sp = float(self.motor_speed)            # start from last valid (hold)
        roller_sp = float(self.sync_roller_speed)

        if len(self._hist_pos_motor_filt) >= 2:
            k = min(5, len(self._hist_pos_motor_filt) - 1)  # 2..6 samples
            dtw = k * fixed_dt

            # motor rev/s
            p_now = self._hist_pos_motor_filt[-1]
            p_old = self._hist_pos_motor_filt[-1 - k]
            if p_now is not None and p_old is not None and dtw > 0:
                motor_sp_candidate = (p_now - p_old) / dtw
                if abs(motor_sp_candidate) <= max_motor_rps * 1.5:
                    motor_sp = motor_sp_candidate  # accept; else hold previous

            # roller rev/s from counts
            r_now = self._hist_pos_roller_filt[-1]
            r_old = self._hist_pos_roller_filt[-1 - k]
            if r_now is not None and r_old is not None and dtw > 0:
                dr_counts = (r_now - r_old)
                roller_sp_candidate = (dr_counts / cpr) / dtw
                if abs(roller_sp_candidate) <= max_roller_rps * 1.5:
                    roller_sp = roller_sp_candidate  # accept; else hold previous

        # brake gating: zero + reseed filtered positions
        if brake_status and (abs(motor_sp) < brake_zero_rps) and (abs(roller_sp) < brake_zero_rps):
            motor_sp = 0.0
            roller_sp = 0.0
            self._pos_motor_filt = motor_unwrapped_rev
            self._pos_roller_filt = roller_unwrapped_counts

        # write filtered speeds back
        self.motor_speed = float(motor_sp)
        self.sync_roller_speed = float(roller_sp)

        # 9) update public state and publish DebugMessage
        self.brake_status = bool(brake_status)
        self.current = current
        self.motor_torque = motor_torque
        self.syncronous_roller_raw_wrapped = sync_raw
        self.motor_position = motor_pos_norm

        motor_speed_rad_s = self.motor_speed * tau2pi
        roller_speed_rad_s = self.sync_roller_speed * tau2pi
        rope_speed_m_s = roller_speed_rad_s * radius_m

        # final rope sanity (and hold)
        if abs(rope_speed_m_s) > phys_max_rope_m_s:
            rope_speed_m_s = 0.0
            roller_speed_rad_s = 0.0
            self.sync_roller_speed = 0.0

        m = DebugMessage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = f"winch_{self.side}"

        def to_f(x):
            try:
                return float(x)
            except Exception:
                return float("nan")

        m.brake = bool(self.brake_status)
        m.current = to_f(self.current) if self.current is not None else float("nan")
        m.motor_torque = to_f(self.motor_torque) if self.motor_torque is not None else float("nan")
        m.syncronous_roller_raw_wrapped = int(self.syncronous_roller_raw_wrapped) if self.syncronous_roller_raw_wrapped is not None else 0
        m.motor_position = to_f(self.motor_position) if self.motor_position is not None else float("nan")

        m.motor_speed_rev_s = to_f(self.motor_speed)
        m.motor_speed_rad_s = to_f(motor_speed_rad_s)
        m.sync_roller_speed_rev_s = to_f(self.sync_roller_speed)
        m.sync_roller_speed_rad_s = to_f(roller_speed_rad_s)
        m.rope_speed_m_s = to_f(rope_speed_m_s)

        self.pub_debug.publish(m)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _load_config(self, path: str) -> Optional[dict]:
        try:
            p = Path(path)
            if not p.exists():
                alt = Path(__file__).resolve().parent.parent / "config" / "arganelloTelemetry.json"
                p = alt if alt.exists() else p
            if not p.exists():
                return None
            with p.open("r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load CONFIG: {e}")
            return None

    def _stdin_loop(self) -> None:
        import sys, select
        prompt = f"[{self.get_name()}:{self.side}]> "
        sys.stdout.write(prompt); sys.stdout.flush()
        while not self._stop:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            line = sys.stdin.readline()
            if not line:
                continue
            line = line.strip()
            if not line:
                sys.stdout.write(prompt); sys.stdout.flush()
                continue
            if line in ("quit", "exit"):
                self.get_logger().info("Exiting debug console…")
                break
            self.send_cmd(line)
            sys.stdout.write(prompt); sys.stdout.flush()

    # ── SYNC helpers ─────────────────────────────────────────────────────────
    def _send_sync(self):
        if self.sync_epoch_unit == "ns":
            epoch = time.time_ns()
        else:
            epoch = int(time.time_ns() // 1_000_000)  # ms since epoch
        cmd = f"sync {epoch}"
        self.get_logger().info(f"→ {cmd}")
        self.send_cmd(cmd)
        return epoch

    def destroy_node(self) -> None:
        self._stop = True
        time.sleep(0.02)
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


    # ── filter helpers ─────────────────────────────────────────────────────────
    def moving_average_filter(self, value, window=5, name="default"):
        """
        Moving-average filter that preserves int/float type and is None-safe.
        - If value is None: return last filtered value for this 'name' (or None if none yet),
        and do NOT update the window.
        """
        # per-signal state names
        hist_name = f"_ma_hist_{name}"
        last_name = f"_ma_last_{name}"

        # init state if missing
        if not hasattr(self, hist_name):
            setattr(self, hist_name, deque(maxlen=max(1, int(window))))
        if not hasattr(self, last_name):
            setattr(self, last_name, None)

        hist: deque = getattr(self, hist_name)
        last = getattr(self, last_name)

        # invalid window => just return input
        if window < 1:
            return value

        # if current sample is None, don't touch history; return last filtered
        if value is None:
            return last  # may be None on first calls

        # normal update
        hist.append(float(value))
        avg = sum(hist) / len(hist)

        # keep same type as input
        filtered = int(round(avg)) if isinstance(value, int) else avg
        setattr(self, last_name, filtered)
        return filtered
    


    def glitch_outlier_filter(self, value, *, name="enc", window=15, k=3.5,
                            max_step=None, persist=2, tol_accept=None):
        """
        Outlier remover for fast-changing signals.
        - Uses Hampel (median + MAD) + optional physical max_step + persistence.
        - On a suspected glitch, it HOLDS the last accepted value.
        - If the big change persists for `persist` samples, it is accepted.

        Args:
        value: int | float | None
        name: separate state per signal
        window: history size for median/MAD
        k: Hampel sensitivity (3.0–3.5 typical)
        max_step: optional absolute per-sample limit (same units as value)
        persist: number of consecutive samples required to accept a big jump
        tol_accept: small tolerance to treat two big samples as "same jump" (defaults to 1% of median or 1.0)

        Returns:
        filtered value with same type as input (int stays int), or None if no history yet.
        """
        hist_name   = f"_gf_hist_{name}"
        last_name   = f"_gf_last_{name}"
        pend_name   = f"_gf_pending_{name}"   # (pending_value, count)
        is_int = isinstance(value, int)

        # None handling: hold last if we have it
        if value is None:
            return getattr(self, last_name, None)

        x = float(value)

        # Ensure per-signal state
        if not hasattr(self, hist_name):
            setattr(self, hist_name, deque(maxlen=max(3, window)))
        hist: deque = getattr(self, hist_name)

        if not hasattr(self, last_name):
            setattr(self, last_name, x)  # first sample becomes last
        if not hasattr(self, pend_name):
            setattr(self, pend_name, (None, 0))

        last = float(getattr(self, last_name))
        pending_val, pending_cnt = getattr(self, pend_name)

        # Helper: robust median & MAD
        def med_mad(values):
            vals = list(values)
            if not vals:
                return x, 0.0
            vs = sorted(vals)
            n = len(vs)
            med = vs[n//2] if n % 2 == 1 else 0.5*(vs[n//2 - 1] + vs[n//2])
            abs_dev = [abs(v - med) for v in vs]
            abs_dev.sort()
            mad = abs_dev[n//2] if n % 2 == 1 else 0.5*(abs_dev[n//2 - 1] + abs_dev[n//2])
            # 1.4826 ≈ scale for normal distribution
            return med, 1.4826 * (mad if mad > 0 else 0.0)

        # Not enough history yet → accept and learn
        if len(hist) < 3:
            hist.append(x)
            setattr(self, last_name, x)
            return int(round(x)) if is_int else x

        med, s = med_mad(hist)
        if tol_accept is None:
            tol_accept = max(0.01 * max(abs(med), 1.0), 1.0)  # ~1% or 1 unit minimum

        # Hampel outlier test
        is_hampel_outlier = (abs(x - med) > k * max(s, 1e-9))

        # Physical step limit (optional)
        is_step_outlier = False
        if max_step is not None:
            is_step_outlier = abs(x - last) > max_step

        # If clearly not an outlier → accept immediately
        if not (is_hampel_outlier or is_step_outlier):
            hist.append(x)
            setattr(self, pend_name, (None, 0))
            setattr(self, last_name, x)
            return int(round(x)) if is_int else x

        # Big change detected → require persistence
        if pending_val is None:
            # start a pending jump
            setattr(self, pend_name, (x, 1))
            # hold last value for now
            return int(round(last)) if is_int else last
        else:
            # If the new big sample is close to the previous pending one, accumulate
            if abs(x - pending_val) <= tol_accept:
                pending_cnt += 1
                setattr(self, pend_name, (pending_val, pending_cnt))
            else:
                # different spike → restart pending
                setattr(self, pend_name, (x, 1))
                pending_cnt = 1
                pending_val = x

            if pending_cnt >= persist:
                # Accept the jump as real
                hist.append(pending_val)
                setattr(self, last_name, pending_val)
                setattr(self, pend_name, (None, 0))
                return int(round(pending_val)) if is_int else pending_val
            else:
                # Still pending → hold last
                return int(round(last)) if is_int else last




def main():
    rclpy.init()
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down…")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

'''
readme content as follows 

ros2 run cl_arganello_interface telemetry_node.py \
  --ros-args \
  -p side:=left \
  -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00 \
  -p config_path:=/home/msi/Desktop/ros2_ws/src/cl_arganello_interface/config/arganelloTelemetry.json \
  -p debug_mode:=true

ros2 service call /winch/left/brake_engage std_srvs/srv/Trigger "{}"
ros2 service call /winch/left/brake_disengage std_srvs/srv/Trigger "{}"

# IDLE
ros2 topic pub --once /winch/left/set_motor_mode std_msgs/msg/String "{data: 'idle'}"
# CLOSED-LOOP TORQUE
ros2 topic pub --once /winch/left/set_motor_mode std_msgs/msg/String "{data: 'closed_loop_torque'}"
# CLOSED-LOOP VELOCITY
ros2 topic pub --once /winch/left/set_motor_mode std_msgs/msg/String "{data: 'closed_loop_velocity'}"
# CLOSED-LOOP POSITION
ros2 topic pub --once /winch/left/set_motor_mode std_msgs/msg/String "{data: 'closed_loop_position'}"








'''