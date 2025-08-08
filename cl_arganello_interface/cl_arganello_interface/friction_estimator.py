#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
from cl_arganello_interface.msg import ArganelloEnhancedTelemetry
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# ─────────────────────────────────────────────────────────────
# Represents a single torque step: (torque in Nm, duration in seconds)
# ─────────────────────────────────────────────────────────────
class TorqueStep:
    def __init__(self, torque: float, duration: float):
        self.torque = torque
        self.duration = duration

# ─────────────────────────────────────────────────────────────
# Main ROS2 Node for Friction Parameter Estimation
# ─────────────────────────────────────────────────────────────
class FrictionEstimatorNode(Node):
    def __init__(self):
        super().__init__('friction_estimator_node')

        # === Setup Output File ===
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f'friction_log_{now}.csv'
        self.csv_header = "timestamp,motor_vel_rad_s,motor_torque\n"
        self.logging_active = False

        # === Define Torque Sequence ===
        self.sequence = [
                    # torque (n/m) duration(s)

            TorqueStep(0.6, 3.0),    # steady
            TorqueStep(+1.0, 0.8),  # up
            TorqueStep(0.4, 3.0),    # steady
            TorqueStep(-0.1, 0.58),    # down
            TorqueStep(0.6, 3.0),    # stop steady

            TorqueStep(0.6, 3.0),    # steady
            TorqueStep(+1.5, 0.315),  # up
            TorqueStep(0.4, 3.0),    # steady
            TorqueStep(-0.1, 0.58),    # down
            TorqueStep(0.6, 3.0),    # stop steady

            TorqueStep(0.6, 3.0),    # steady
            TorqueStep(+2.0, 0.21),  # up
            TorqueStep(0.4, 3.0),    # steady
            TorqueStep(-0.1, 0.58),    # down
            TorqueStep(0.6, 3.0),    # stop steady


        ]
        self.current_step_index = -1
        self.step_start_time = self.get_clock().now()

        # === ROS Communication ===
        self.publisher = self.create_publisher(Float32, '/arganello/dx/target_torque', 10)
        self.subscription = self.create_subscription(
            ArganelloEnhancedTelemetry,
            '/arganello/dx/telemetry/enhanced',
            self.telemetry_callback,
            10
        )
        self.timer = self.create_timer(0.01, self.step1_ramp_callback)

        # === Start Logging ===
        self.get_logger().info("═══════════════════════════════════════")
        self.get_logger().info("📌 STEP 1: Applying torque ramp...")
        self.get_logger().info("📌 STEP 2: Logging telemetry to CSV...")
        self.get_logger().info("═══════════════════════════════════════")
        self.file = open(self.filename, 'w')
        self.file.write(self.csv_header)
        self.logging_active = True

    # ─────────────────────────────────────────────────────────────
    # STEP 1: Torque Ramp Callback
    # ─────────────────────────────────────────────────────────────
    def step1_ramp_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.step_start_time).nanoseconds / 1e9

        if self.current_step_index == -1 or elapsed >= self.sequence[self.current_step_index].duration:
            self.current_step_index += 1

            if self.current_step_index >= len(self.sequence):
                self.publisher.publish(Float32(data=0.6))
                self.logging_active = False
                self.timer.cancel()
                self.file.close()
                self.get_logger().info("✅ STEP 1 Complete: Torque ramp ended.")
                self.get_logger().info("✅ STEP 2 Complete: CSV logging ended.")
                self.get_logger().info(f"📂 Data saved to: {self.filename}")
                self.get_logger().info("═══════════════════════════════════════")
                self.get_logger().info("📌 STEP 3: Reloading CSV & estimating parameters...")
                self.get_logger().info("═══════════════════════════════════════")
                self.step3_estimate_parameters()
                return

            step = self.sequence[self.current_step_index]
            self.publisher.publish(Float32(data=step.torque))
            self.step_start_time = now
            self.get_logger().info(f"[Step {self.current_step_index+1}/{len(self.sequence)}] τ = {step.torque:.2f} Nm for {step.duration:.2f}s")

    # ─────────────────────────────────────────────────────────────
    # STEP 2: Telemetry Callback
    # ─────────────────────────────────────────────────────────────
    def telemetry_callback(self, msg: ArganelloEnhancedTelemetry):
        if not self.logging_active:
            return
        try:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            motor_vel = msg.motor_vel * 2 * np.pi  # rad/s
            torque = msg.motor_torque
            self.file.write(f"{timestamp:.6f},{motor_vel:.6f},{torque:.6f}\n")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Telemetry logging failed: {e}")

    # STEP 3 ────────────────────────────────────────────────────────
    # Load CSV and estimate friction parameters via linear regression
    # ───────────────────────────────────────────────────────────────
    def step3_estimate_parameters(self):
        try:
            self.get_logger().info("📌 STEP 3: Estimating friction parameters via linear regression...")

            # Load telemetry data from CSV
            timestamps, velocities, torques = [], [], []
            with open(self.filename, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # Skip header
                for row in reader:
                    ts, vel, tq = map(float, row)
                    timestamps.append(ts)
                    velocities.append(vel)
                    torques.append(tq)

            # Convert lists to numpy arrays
            vel_arr = np.array(velocities)
            torque_arr = np.array(torques)

            # Constants (update if available)
            I = 0.0                # Inertia [kg·m²]
            theta_ddot = 0.0       # Angular acceleration [rad/s²]
            tau_gravity = 0.0      # Gravitational torque [Nm]

            mass = 1.55  # mass in kg 
            g = 9.81    # acceleration m/s^2
            r = 0.020 + 0.0125  # effectve radiu of the winxh with 3 turns and a 1:1 gear ration to the motor (m) 
                                # beacuse in out test the rope stays warpe 3 times we can assume it constant for better accuracy. 
                                # use varibale gear ration otherwise
            tau_gravity = mass * g * r 
            # Compute friction torque for regression
            tau_friction = torque_arr - I * theta_ddot - tau_gravity

            # Search for best P3 (Striebeck threshold) over a range
            theta_th_values = np.linspace(0.1, 3.0, 200)
            best_error = float('inf')
            best_params = None
            best_theta_th = None

            # ────────────────────────────────────────
            # Generate timestamped debug filename
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            debug_filename = f"debug_regression_{timestamp}.csv"
            # ────────────────────────────────────────

            with open(debug_filename, "w") as dbg:
                dbg.write("theta_th,P1,P2,P4,error\n")

                for theta_th in theta_th_values:
                    sgn = np.sign(vel_arr)
                    Ei = np.exp(-np.abs(vel_arr) / theta_th)  # Striebeck effect

                    # Build regression matrix A
                    A = np.column_stack((
                        sgn,          # ➜ P1 (Coulomb friction)
                        sgn * Ei,     # ➜ P2 (Stiction exponential decay)
                        vel_arr       # ➜ P4 (Viscous friction)
                    ))

                    y = tau_friction
                    x, *_ = np.linalg.lstsq(A, y, rcond=None)
                    residual = y - A @ x
                    error = np.linalg.norm(residual)

                    # Write to debug CSV
                    dbg.write(f"{theta_th:.6f},{x[0]:.6f},{x[1]:.6f},{x[2]:.6f},{error:.6f}\n")

                    if error < best_error:
                        best_error = error
                        best_params = x
                        best_theta_th = theta_th

            # Extract best-fit parameters
            P1, P2, P4 = best_params
            P3 = best_theta_th

            # Report results
            self.get_logger().info("✅ STEP 3 Complete: Model fitted.")
            self.get_logger().info(f"   P1 = {P1:.4f} Nm (Coulomb friction)")
            self.get_logger().info(f"   P2 = {P2:.4f} Nm (Stiction component)")
            self.get_logger().info(f"   P3 = {P3:.4f} rad/s (Striebeck threshold)")
            self.get_logger().info(f"   P4 = {P4:.4f} Nm⋅s/rad (Viscous coefficient)")
            self.get_logger().info(f"📄 Regression debug saved to: {debug_filename}")

            # Proceed to plotting
            self.step4_plot_results(vel_arr, torque_arr, P1, P2, P3, P4)

        except Exception as e:
            self.get_logger().error(f"🚫 STEP 3 Failed: {e}")



    # ─────────────────────────────────────────────────────────────
    # STEP 4: Plot measured vs. model data
    # ─────────────────────────────────────────────────────────────
    def step4_plot_results(self, vel_arr, torque_arr, P1, P2, P3, P4):
        try:
            self.get_logger().info("═══════════════════════════════════════")
            self.get_logger().info("📌 STEP 4: Plotting model vs measured data...")
            self.get_logger().info("   ➤ Fitted Friction Parameters:")
            self.get_logger().info(f"     • P1 = {P1:.4f} Nm (Coulomb friction)")
            self.get_logger().info(f"     • P2 = {P2:.4f} Nm (Stiction magnitude)")
            self.get_logger().info(f"     • P3 = {P3:.4f} rad/s (Striebeck threshold)")
            self.get_logger().info(f"     • P4 = {P4:.4f} Nm⋅s/rad (Viscous coefficient)")
            self.get_logger().info("═══════════════════════════════════════")

            # Model prediction at measured points
            predicted = np.sign(vel_arr) * (P1 + P2 * np.exp(-np.abs(vel_arr) / P3)) + P4 * vel_arr

            # Smooth line for model
            v_dense = np.linspace(min(vel_arr), max(vel_arr), 1000)
            model_dense = np.sign(v_dense) * (P1 + P2 * np.exp(-np.abs(v_dense) / P3)) + P4 * v_dense

            # ──────────────── PLOT 1: Friction Model Fit ────────────────
            plt.figure(figsize=(10, 6))
            plt.plot(vel_arr, torque_arr, 'b.', label='Measured Torque (Nm)', alpha=0.5)
            plt.plot(v_dense, model_dense, 'r-', linewidth=2, label='Fitted Friction Model')
            plt.xlabel("Angular Velocity $\\dot{\\theta}$ [rad/s]")
            plt.ylabel("Torque $\\tau$ [Nm]")
            plt.title("Friction Model Fit\n"
                    f"P1={P1:.3f} Nm, P2={P2:.3f} Nm, P3={P3:.3f} rad/s, P4={P4:.4f} Nm⋅s/rad")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.show()

            # ──────────────── PLOT 2: Residuals ────────────────
            residuals = torque_arr - predicted
            plt.figure(figsize=(10, 4))
            plt.plot(vel_arr, residuals, 'k.', alpha=0.5)
            plt.axhline(0, color='gray', linestyle='--', linewidth=1)
            plt.xlabel("Angular Velocity $\\dot{\\theta}$ [rad/s]")
            plt.ylabel("Residual Torque $\\tau_{measured} - \\tau_{model}$ [Nm]")
            plt.title("Model Residuals (Torque Error vs Velocity)")
            plt.grid(True)
            plt.tight_layout()
            plt.show()

            self.get_logger().info("✅ STEP 4 Complete: Plots shown.")

        except Exception as e:
            self.get_logger().error(f"🚫 STEP 4 Failed: {e}")

    def destroy_node(self):
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()
        super().destroy_node()

# ─────────────────────────────────────────────────────────────
# Main Execution Entry Point
# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = FrictionEstimatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
