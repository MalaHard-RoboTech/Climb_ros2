#!/usr/bin/env python3
'''
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger          # empty request/response service
from std_msgs.msg import Float32


class JumpNode(Node):
    def __init__(self):
        super().__init__('jump_node')

        # Publishers at 100 Hz
        self.pub_s1 = self.create_publisher(Float32, '/alpine/dongle/servoValve1', 10)
        self.pub_s2 = self.create_publisher(Float32, '/alpine/dongle/servoValve2', 10)
        self.timer = self.create_timer(0.01, self.publish_default)  # 100 Hz

        # Jump sequence state
        self.sequence_running = False
        self.sequence_start = None

        # Define jump sequence as durations (ms) → angles
        # Format: (duration_ms, s1, s2)
        #s2 aria in
        #s1 aria out 
        self.sequence = [
            (600,   0.0, 90.0),   # Phase 1: hold 800 ms
            (10,   0.0, 0.0),    # Phase 2: hold 100 ms
            (600,  90.0, 0.0),    # Phase 3: hold 600 ms
            #(100,  90.0, 0.0),    # Phase 4: hold 100 ms
            #(400,   0.0, 0.0),    # Phase 5: hold 400 ms
        ]

        # Precompute cumulative timeline for efficiency
        self.timeline = []
        t = 0
        for dur, s1, s2 in self.sequence:
            t += dur
            self.timeline.append((t, s1, s2))

        # Service to trigger jump
        self.create_service(Trigger, '/alpine/jump', self.handle_jump)

        self.get_logger().info("jump_node started, publishing 0 deg on both valves at 100 Hz")

    def publish_default(self):
        now = self.get_clock().now()
        if not self.sequence_running:
            # normal idle publish (0 deg on both valves)
            self.pub_s1.publish(Float32(data=0.0))
            self.pub_s2.publish(Float32(data=0.0))
            return

        # elapsed time in ms since sequence start
        elapsed = (now.nanoseconds - self.sequence_start) / 1e6

        # Default angles
        s1, s2 = 0.0, 0.0

        # Walk through timeline and pick the correct step
        for limit, a1, a2 in self.timeline:
            if elapsed < limit:
                s1, s2 = a1, a2
                break
        else:
            # Sequence ended
            self.sequence_running = False
            self.get_logger().info("Jump sequence completed")

        # Publish at 100 Hz
        self.pub_s1.publish(Float32(data=s1))
        self.pub_s2.publish(Float32(data=s2))

    def handle_jump(self, request, response):
        self.sequence_running = True
        self.sequence_start = self.get_clock().now().nanoseconds
        self.get_logger().info("Jump sequence triggered")
        response.success = True
        response.message = "Jump sequence started"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = JumpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

'''

# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Trigger
# from std_msgs.msg import Float32
# from cl_arganello_interface.msg import RopeCommand 
# import time

# class JumpNode(Node):
#     def __init__(self):
#         super().__init__('jump_node')

#         # ── Publishers at 100 Hz ──────────────────────
#         self.pub_s1 = self.create_publisher(Float32, '/alpine/dongle/servoValve1', 2)
#         self.pub_s2 = self.create_publisher(Float32, '/alpine/dongle/servoValve2', 2)
#         self.pub_left = self.create_publisher(RopeCommand, '/winch/left/command', 2)
#         self.pub_right = self.create_publisher(RopeCommand, '/winch/right/command', 2)
#         self.timer = self.create_timer(0.02, self.publish_default)  # 50 Hz

#         # ── Jump sequence state ───────────────────────
#         self.sequence_running = False
#         self.sequence_start = None

#         # ── Jump sequence (duration_ms, left_force, right_force, s1, s2) ──────
#         self.sequence = [
#             (200,  -15.0,  15.0,   0.0, 90.0),   # Phase 1
#             (5,   -15.0,  15.0,   0.0,  0.0),   # Phase 2
#             (2000,  -15.0,  15.0,   90.0,  0.0),   # Phase 3

#             (3000,  -8.0,  8.0,   90.0,  0.0),   # Phase 4
#             (2000,  -8.0,  8.0,   0.0,  0.0),   # Phase 4

#             (300,  -2.0,  2.0,   0.0, 90.0),   # Phase 5
#             (10,   -2.0,  2.0,   0.0,  0.0),   # Phase 6
#             (2000,  -8.0,  8.0,   90.0,  0.0),   # Phase 7

#             (5000,  -8.0,  8.0,   90.0,  0.0),   # Phase 4
#         ]

#         # Precompute cumulative timeline
#         self.timeline = []
#         t = 0
#         for dur, lf, rf, s1, s2 in self.sequence:
#             t += dur
#             self.timeline.append((t, lf, rf, s1, s2))

#         # ── Service to trigger jump ───────────────────
#         self.create_service(Trigger, '/alpine/jump', self.handle_jump)

#         self.get_logger().info("jump_node started, publishing idle at 100 Hz")

#     # ── Publish helper (ensures sync) ─────────────────────────────
#     def publish_all(self, lf: float, rf: float, s1: float, s2: float):
#         """Publish motors and valves together in one call."""

#         # Valves first
#         self.pub_s1.publish(Float32(data=s1))
#         self.pub_s2.publish(Float32(data=s2))
#         time.sleep(0.5)
#         # Motors after
#         rope_left = RopeCommand(rope_force=lf, rope_velocity=0.0, rope_position=0.0)
#         rope_right = RopeCommand(rope_force=rf, rope_velocity=0.0, rope_position=0.0)
#         self.pub_left.publish(rope_left)
#         self.pub_right.publish(rope_right)


#     # ── Main 100 Hz callback ─────────────────────────
#     def publish_default(self):
#         now = self.get_clock().now()
#         if not self.sequence_running:
#             # Idle safety mode
#             self.publish_all(-8.0, 8.0, 0.0, 0.0)
#             return

#         # elapsed ms
#         elapsed = (now.nanoseconds - self.sequence_start) / 1e6

#         # defaults
#         lf, rf, s1, s2 = -8.0, 8.0, 0.0, 0.0

#         # find current step
#         for limit, lf_val, rf_val, s1_val, s2_val in self.timeline:
#             if elapsed < limit:
#                 lf, rf, s1, s2 = lf_val, rf_val, s1_val, s2_val
#                 break
#         else:
#             # sequence finished
#             self.sequence_running = False
#             self.get_logger().info("Jump sequence completed")

#         # Publish all commands at the same time
#         self.publish_all(lf, rf, s1, s2)

#     def handle_jump(self, request, response):
#         self.sequence_running = True
#         self.sequence_start = self.get_clock().now().nanoseconds
#         self.get_logger().info("Jump sequence triggered")
#         response.success = True
#         response.message = "Jump sequence started"
#         return response


# def main(args=None):
#     rclpy.init(args=args)
#     node = JumpNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from cl_arganello_interface.msg import RopeCommand

MS = 1.0  # readability

class JumpNode(Node):
    def __init__(self):
        super().__init__('jump_node')

        # ── Publishers ──────────────────────────────────────────────
        self.pub_s1 = self.create_publisher(Float32, '/alpine/dongle/servoValve1', 10) 
        self.pub_s2 = self.create_publisher(Float32, '/alpine/dongle/servoValve2', 10) 
        self.pub_left  = self.create_publisher(RopeCommand, '/winch/left/command', 10) 
        self.pub_right = self.create_publisher(RopeCommand, '/winch/right/command',10) 

        # ── Timer: 1 ms state machine (non-blocking) ───────────────
        # Cheap work per tick; we only check deadlines and publish if due.
        self.timer = self.create_timer(0.001, self.tick)  # 1 kHz

        # ── Jump sequence state ─────────────────────────────────────
        self.sequence_running = False
        self.sequence_start_ms = 0.0
        self.pending_motor_deadline_ms = None  # set when we publish valves

        self.up_force = 18.0
        self.down_force = 6.0
        self.hold_force = 10.0
        # Sequence: (duration_ms, left_force, right_force, s1, s2)
        self.sequence = [
            
            
            (200,  -self.up_force,  self.up_force,   90.0, 0.0),   # Phase 1 salto
            (5,   -self.up_force,  self.up_force,   0.0,  0.0),   # Phase 2 volo
            (1200,  -self.up_force,  self.up_force,   0.0,  90.0),   # Phase 3 atterraggio
            
            (3000,  -self.hold_force,  self.hold_force,   0.0,  0.0),   # Phase 4 fermo
            
            
            (200,  -self.down_force,  self.down_force,   90.0, 0.0),   # Phase 1 salto
            (5,   -self.down_force,  self.down_force,   0.0,  0.0),   # Phase 2 volo
            (1600,  -self.down_force,  self.down_force,   0.0,  90.0),   # Phase 3 atterraggio
            
            (1000,  -self.hold_force,  self.hold_force,   0.0,  0.0),   # Phase 4 fermo
            
            (1000,  -self.hold_force,  self.hold_force,   0.0,  90.0),   # Phase 3 reset
            (1000,  -self.hold_force,  self.hold_force,   0.0,  0.0),   # Phase 4 fermo
            
        ]

        # Precompute cumulative timeline in ms
        self.timeline = []
        acc = 0
        for dur, lf, rf, s1, s2 in self.sequence:
            acc += dur
            self.timeline.append((acc, lf, rf, s1, s2))

        # Service to trigger jump
        self.create_service(Trigger, '/alpine/jump', self.handle_jump)

        self.get_logger().info("jump_node started (1 kHz state machine, no blocking sleeps)")

        # Idle defaults
        self.idle_lf = -self.hold_force
        self.idle_rf =  self.hold_force
        self.idle_s1 =  0.0
        self.idle_s2 =  0.0

    # Monotonic ms helper
    def now_ms(self) -> float:
        return self.get_clock().now().nanoseconds / 1e6

    # Non-blocking: publish valves immediately, schedule motors for t+500 ms
    def publish_valves_then_motors(self, lf: float, rf: float, s1: float, s2: float):
        # 1) Valves now
        self.pub_s1.publish(Float32(data=s1))
        self.pub_s2.publish(Float32(data=s2))

        # 2) Schedule motors at +500 ms (no sleep)
        self.pending_motor_deadline_ms = self.now_ms() + 500.0

        # Stash pending motor values
        self.pending_lf = lf
        self.pending_rf = rf

    # Called at 1 kHz; do tiny work each tick
    def tick(self):
        t_ms = self.now_ms()

        # If we have a scheduled motor publish, fire it when due
        if self.pending_motor_deadline_ms is not None and t_ms >= self.pending_motor_deadline_ms:
            rope_left  = RopeCommand(rope_force=self.pending_lf, rope_velocity=0.0, rope_position=0.0)
            rope_right = RopeCommand(rope_force=self.pending_rf, rope_velocity=0.0, rope_position=0.0)
            self.pub_left.publish(rope_left)
            self.pub_right.publish(rope_right)
            self.pending_motor_deadline_ms = None  # clear

        # Idle mode
        if not self.sequence_running:
            # Keep outputs refreshed but lightweight
            # Only issue commands if we don’t have a pending motor publish
            if self.pending_motor_deadline_ms is None:
                self.publish_valves_then_motors(self.idle_lf, self.idle_rf, self.idle_s1, self.idle_s2)
            return

        # Sequence mode: compute current step from elapsed
        elapsed_ms = t_ms - self.sequence_start_ms

        # defaults if beyond end
        lf, rf, s1, s2 = self.idle_lf, self.idle_rf, self.idle_s1, self.idle_s2

        for limit_ms, lf_v, rf_v, s1_v, s2_v in self.timeline:
            if elapsed_ms < limit_ms:
                lf, rf, s1, s2 = lf_v, rf_v, s1_v, s2_v
                break
        else:
            self.sequence_running = False
            self.get_logger().info("Jump sequence completed")

        # Only schedule a new pair if no motor publish is pending (keeps ordering)
        if self.pending_motor_deadline_ms is None:
            self.publish_valves_then_motors(lf, rf, s1, s2)

    def handle_jump(self, request, response):
        self.sequence_running = True
        self.sequence_start_ms = self.now_ms()
        self.pending_motor_deadline_ms = None  # reset any pending
        self.get_logger().info("Jump sequence triggered")
        response.success = True
        response.message = "Jump sequence started"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = JumpNode()

    # Multi-threaded executor so timers & services don’t block each other
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
