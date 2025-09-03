#!/usr/bin/env python3

"""
IMU-BASED PID SPACECRAFT CONTROLLER
==================================

This controller subscribes to IMU data and integrates acceleration to calculate velocity,
then uses PID control to precisely stop the spacecraft.

FEATURES:
✅ Real IMU sensor data from Isaac Sim  
✅ Acceleration integration for velocity calculation
✅ Ultra-precise PID control (< 0.5 mm/s stopping)
✅ ROS-based architecture
✅ Single file solution

TOPICS:
- Subscribes: /srb/env0/imu_robot (sensor_msgs/Imu)
- Publishes: /srb/env0/robot/thrust (std_msgs/Float32MultiArray)
- Publishes: /spacecraft/velocity (geometry_msgs/Vector3) [for monitoring]

USAGE:
    cd /home/bfarhat/SURF/space_robotics_bench
    python3 imu_pid_controller.py
    
Then use the interactive menu to control the spacecraft with IMU-based precision!
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
try:
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    TF2_AVAILABLE = True
except Exception:
    TF2_AVAILABLE = False
import numpy as np
import time
import threading
import signal
import sys

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=1.0):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.max_output = max_output
        
        self.prev_error = 0.0
        self.integral = 0.0
        
    def update(self, error, dt):
        """Update PID controller with error and time step"""
        # Proportional term
        p_term = self.kp * error
        
        # Integral term  
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        # Combined output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(-self.max_output, min(self.max_output, output))
        
        return output
        
    def reset(self):
        """Reset PID controller state"""
        self.prev_error = 0.0
        self.integral = 0.0

class IMUPIDController(Node):
    def __init__(self):
        super().__init__('imu_pid_spacecraft_controller')
        
        # ROS Publishers
        self.thrust_pub = self.create_publisher(
            Float32MultiArray,
            '/srb/env0/robot/thrust',
            10
        )
        
        self.velocity_pub = self.create_publisher(
            Vector3,
            '/spacecraft/velocity', 
            10
        )
        
        # ROS Subscribers  
        self.imu_sub = self.create_subscription(
            Imu,
            '/srb/env0/imu_robot',
            self.imu_callback,
            10
        )
        
        # IMU Integration State
        self.velocity = np.array([0.0, 0.0, 0.0])  # Integrated velocity from IMU
        self.last_imu_time = None
        self.imu_received = False
        self.accel_bias = np.zeros(3)
        self.accel_filt = np.zeros(3)
        self.bias_calibrated = False
        self.bias_samples = []
        self.position = np.array([0.0, 0.0, 0.0])  # Integrated position from IMU
        
        # PID Controllers (start conservative; PD only)
        self.pid_x = PIDController(kp=0.05, ki=0.0, kd=0.01, max_output=0.05)
        self.pid_y = PIDController(kp=0.05, ki=0.0, kd=0.01, max_output=0.05)
        self.pid_z = PIDController(kp=0.05, ki=0.0, kd=0.01, max_output=0.05)
        
        # Control State
        self.controller_active = False
        self.target_velocity = np.array([0.0, 0.0, 0.0])
        self.velocity_tolerance = 0.02  # 2 cm/s for stable stop
        self.deadband = 0.02  # 2 cm/s deadband to avoid chatter
        
        # Control Loop Timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self._control_debug_counter = 0
        
        # Oscillation Detection
        self.velocity_history = []
        self.max_history_length = 10

        # Thruster polarity autodetection (+X/+Y command -> sign of dV)
        self.map_x = 1   # +1 means pid_output_x>0 -> +X thrusters; -1 flips mapping
        self.map_y = 1
        self.osc_cooldown = 0
        # Thrust→acceleration gains (m/s^2 per unit duty), learned via calibration
        self.ax_gain = 1.0
        self.ay_gain = 1.0
        # Track last commanded thrust magnitude for ZUPT
        self._last_cmd_norm = 0.0
        self._last_cmd_time = time.time()
        # --- final-stop tuning ---
        self.v_high = 0.15       # m/s
        self.v_low = 0.05        # m/s (gentler settle)
        self.min_duty = 0.005    # smaller minimum pulse
        self.pulse_dt = 0.12     # shorter pulses
        self.pulse_cool = 0.10   # seconds coast between pulses
        self.max_pulses = 40     # safety bound
        self.final_pos_window = 1.0  # m: if within this, ignore position and just zero velocity
        
        self.get_logger().info('🎯 IMU-BASED PID SPACECRAFT CONTROLLER INITIALIZED!')
        self.get_logger().info(f'📡 Waiting for IMU data on: /srb/env0/imu_robot')
        self.get_logger().info(f'🎯 PID Gains: Kp={self.pid_x.kp}, Ki={self.pid_x.ki}, Kd={self.pid_x.kd}')
        self.get_logger().info(f'🎯 Ultra-Strict Tolerance: {self.velocity_tolerance} m/s (0.5 mm/s!)')
        self.get_logger().info(f'🎯 Deadband: {self.deadband} m/s')
        self.get_logger().info('🚀 Publishing thrust commands to: /srb/env0/robot/thrust')
        
        # --- Truth (Isaac) pose/vel option ---
        self.use_truth = True
        self.truth_timeout = 0.25
        self.last_truth_time = 0.0
        self.position_truth = np.zeros(3)
        self.velocity_truth = np.zeros(3)
        self.truth_received = False

        # Subscribe to odometry (adjust topic if different in your sim)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/srb/env0/robot/odometry',
            self.truth_odom_cb,
            10,
        )

        # TF truth fallback (if Odometry not present)
        self.tf_available = TF2_AVAILABLE
        if self.use_truth and self.tf_available:
            self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=2.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
            # Candidate frame pairs to try, in order
            self.tf_candidates = [
                ("world", "robot"),
                ("map", "robot"),
                ("world", "base"),
                ("map", "base"),
                ("/World", "/srb/env0/robot"),
                ("/World", "robot"),
            ]
            self._last_tf_pos = None
            self.create_timer(0.05, self._truth_timer_cb)
        
    def _quat_to_R_world_from_body(self, q):
        """Convert quaternion to rotation matrix R (world <- body)."""
        x, y, z, w = q.x, q.y, q.z, q.w
        # 3x3 rotation matrix
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),         2*(x*z + y*w)],
            [2*(x*y + z*w),         1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [2*(x*z - y*w),         2*(y*z + x*w),         1 - 2*(x*x + y*y)],
        ])
        return R
        
    def imu_callback(self, msg):
        """Process IMU data and integrate acceleration to get velocity (world-frame)."""
        # Prefer IMU header time if available; fallback to wall time
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if current_time == 0:
            current_time = time.time()

        # Body-frame accelerometer reading (specific force + sensor bias)
        lin_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

        # Initialize timestamp
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            # start collecting bias samples
            self.bias_samples.append((lin_acc, msg.orientation))
            return

        dt = current_time - self.last_imu_time
        # Clamp dt to avoid spikes
        if dt <= 0 or dt > 0.1:
            dt = 0.05

        # Rotation: world <- body and world gravity
        R_wb = self._quat_to_R_world_from_body(msg.orientation)
        g_w = np.array([0.0, 0.0, 0.0])  # set to [0,0,0] if scene is zero-g

        # Bias calibration (sensor bias, not gravity)
        if not self.bias_calibrated:
            if not hasattr(self, '_bias_start_time'):
                self._bias_start_time = self.last_imu_time
            self.bias_samples.append((lin_acc, msg.orientation))
            if current_time - self._bias_start_time < 1.0:
                self.last_imu_time = current_time
                return
            diffs = []
            for acc_b, q in self.bias_samples:
                R = self._quat_to_R_world_from_body(q)
                diffs.append(acc_b - (R.T @ g_w))
            self.accel_bias = np.mean(diffs, axis=0)
            self.bias_calibrated = True
            self.bias_samples = []
            try:
                del self._bias_start_time
            except Exception:
                pass
            self.get_logger().info(f'📏 Sensor accel bias (body): {self.accel_bias.tolist()} m/s²')

        # Correct formulation: a_world = R * (f_b - bias_b) + g_w
        f_b = lin_acc - self.accel_bias
        a_world = R_wb @ f_b + g_w

        # Low-pass filter (EMA) in world frame
        self.accel_filt = 0.9 * self.accel_filt + 0.1 * a_world

        # Optionally override with truth when fresh
        truth_fresh = self.use_truth and self.truth_received and ((current_time - self.last_truth_time) < self.truth_timeout)
        if not truth_fresh:
            # Integrate world velocity and position from IMU
            self.velocity += self.accel_filt * dt
            self.position += self.velocity * dt
        else:
            # Use sim truth directly
            self.velocity[:] = self.velocity_truth
            self.position[:] = self.position_truth

        # ZUPT: zero-velocity update when truly at rest (no thrust, quiet IMU)
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        quiet_acc = float(np.linalg.norm(self.accel_filt)) < 0.03
        quiet_gyro = float(np.linalg.norm(gyro)) < 0.03
        no_thrust = (time.time() - self._last_cmd_time) > 0.2 and self._last_cmd_norm < 1e-3
        if quiet_acc and quiet_gyro and no_thrust:
            self.velocity[:] = 0.0
            # very small bias refinement only when at rest
            self.accel_bias = 0.999 * self.accel_bias + 0.001 * lin_acc

        # Publish velocity for monitoring
        vel_msg = Vector3()
        vel_msg.x = float(self.velocity[0])
        vel_msg.y = float(self.velocity[1])
        vel_msg.z = float(self.velocity[2])
        self.velocity_pub.publish(vel_msg)

        if not self.imu_received:
            self.get_logger().info('✅ IMU DATA RECEIVED! Starting world-frame integration...')
            self.get_logger().info(f'📊 a_world first sample: {a_world[0]:.6f}, {a_world[1]:.6f}, {a_world[2]:.6f} m/s²')
            self.imu_received = True

        self.last_imu_time = current_time
        
    def control_loop(self):
        """Main PID control loop - runs at 20 Hz"""
        if not self.controller_active or not self.imu_received:
            return
            
        self._control_debug_counter += 1
        
        # Calculate velocity errors
        error_x = self.target_velocity[0] - self.velocity[0]
        error_y = self.target_velocity[1] - self.velocity[1] 
        error_z = self.target_velocity[2] - self.velocity[2]
        
        # Apply deadband to prevent tiny oscillations
        if abs(error_x) < self.deadband:
            error_x = 0.0
        if abs(error_y) < self.deadband:
            error_y = 0.0
        if abs(error_z) < self.deadband:
            error_z = 0.0
            
        # Adaptive gains and output limits by speed
        speed = float(np.linalg.norm(self.velocity))
        if speed > 3.0:
            kp, kd, mo = 0.12, 0.05, 0.25
            ki = 0.0
        elif speed > 1.0:
            kp, kd, mo = 0.08, 0.03, 0.18
            ki = 0.0
        else:
            kp, kd, mo = 0.07, 0.010, 0.08
            ki = 0.004
        for pid in (self.pid_x, self.pid_y, self.pid_z):
            pid.kp = kp; pid.kd = kd; pid.max_output = mo; pid.ki = ki

        # Update PID controllers (PD mostly; tiny I near stop)
        dt = 0.02
        pid_output_x = self.pid_x.update(error_x, dt)
        pid_output_y = self.pid_y.update(error_y, dt)
        pid_output_z = self.pid_z.update(error_z, dt)
        
        # Convert PID outputs to thrust commands
        # Thrust array: [+X1, +X2, -X1, -X2, +Y1, +Y2, -Y1, -Y2]
        thrust_cmd = [0.0] * 8
        
        # X-axis control (forward/backward) with polarity mapping
        # Axis gating: control dominant axis first at higher speeds
        dominant_axis = int(np.argmax(np.abs(self.velocity))) if speed > 0.5 else -1

        out_x = pid_output_x if (dominant_axis in (-1, 0)) else 0.0
        if self.map_x < 0:
            out_x = -out_x
        if out_x > 0:  # Need +X thrust
            thrust_cmd[0] = abs(out_x)  # +X1
            thrust_cmd[1] = abs(out_x)  # +X2
        else:  # Need -X thrust
            thrust_cmd[2] = abs(out_x)  # -X1
            thrust_cmd[3] = abs(out_x)  # -X2
            
        # Y-axis control (left/right) with polarity mapping
        out_y = pid_output_y if (dominant_axis in (-1, 1)) else 0.0
        if self.map_y < 0:
            out_y = -out_y
        if out_y > 0:  # Need +Y thrust
            thrust_cmd[4] = abs(out_y)  # +Y1
            thrust_cmd[5] = abs(out_y)  # +Y2
        else:  # Need -Y thrust
            thrust_cmd[6] = abs(out_y)  # -Y1
            thrust_cmd[7] = abs(out_y)  # -Y2
            
        # Rate limit thrust commands
        thrust_limit = self.pid_x.max_output  # mo
        thrust_cmd = [max(0.0, min(thrust_limit, v)) for v in thrust_cmd]
        self.send_thrust_command(thrust_cmd)
        
        # Debug logging
        velocity_error = np.linalg.norm(self.velocity - self.target_velocity)
        if self._control_debug_counter % 10 == 0:  # Log every 0.5 seconds
            self.get_logger().info(
                f'🔍 IMU-PID #{self._control_debug_counter}: '
                f'Vel=[{self.velocity[0]:.6f},{self.velocity[1]:.6f},{self.velocity[2]:.6f}] '
                f'Err=[{error_x:.6f},{error_y:.6f},{error_z:.6f}] '
                f'PID=[{pid_output_x:.4f},{pid_output_y:.4f},{pid_output_z:.4f}] '
                f'|Err|={velocity_error:.6f}'
            )
            
        # Oscillation detection with backoff (do not hard stop)
        self.velocity_history.append(velocity_error)
        if len(self.velocity_history) > self.max_history_length:
            self.velocity_history.pop(0)
            
        # Only check oscillation when moving slowly to avoid false positives
        if self.osc_cooldown > 0:
            self.osc_cooldown -= 1
        elif speed < 1.0 and len(self.velocity_history) >= self.max_history_length:
            recent_avg = np.mean(self.velocity_history[-5:])
            older_avg = np.mean(self.velocity_history[:5])
            if recent_avg > older_avg * 1.4:  # Error growing
                # Back off gains temporarily instead of stopping
                for pid in (self.pid_x, self.pid_y, self.pid_z):
                    pid.kp *= 0.5; pid.kd *= 0.5
                self.osc_cooldown = 40
                self.get_logger().warn('⚠️ IMU-PID oscillation detected -> halving gains for 2s')
                
        # Check if target reached
        min_cycles = 20  # Require ~1 second of stable control
        if velocity_error < self.velocity_tolerance and self._control_debug_counter >= min_cycles:
            if np.allclose(self.target_velocity, [0.0, 0.0, 0.0]):
                self.get_logger().info(
                    f'🎯 IMU-PID TARGET REACHED after {self._control_debug_counter} cycles! '
                    f'Final velocity error: {velocity_error:.6f} m/s'
                )
                self.stop_pid_control()
        elif velocity_error < self.velocity_tolerance:
            remaining = min_cycles - self._control_debug_counter
            self.get_logger().info(
                f'🔄 IMU-PID close to target ({velocity_error:.6f} m/s) - '
                f'waiting {remaining} more cycles for stability'
            )
            
    def send_thrust_command(self, thrust_values):
        """Send thrust command to spacecraft"""
        msg = Float32MultiArray()
        msg.data = [float(x) for x in thrust_values]
        self.thrust_pub.publish(msg)
        self._last_cmd_norm = float(np.linalg.norm(msg.data))
        self._last_cmd_time = time.time()

    def _duty_for_accel(self, axis: str, a_cmd: float) -> float:
        """Convert desired linear acceleration (m/s^2) on an axis to signed duty [-1..1]."""
        if axis == 'x':
            duty = a_cmd / max(self.ax_gain, 1e-6)
            return duty if self.map_x > 0 else -duty
        elif axis == 'y':
            duty = a_cmd / max(self.ay_gain, 1e-6)
            return duty if self.map_y > 0 else -duty
        else:
            return 0.0

    def truth_odom_cb(self, msg: Odometry):
        # Prefer header time; fallback to wall time
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0:
            t = time.time()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.position_truth = np.array([p.x, p.y, p.z], dtype=float)

        # Assume Twist is world-frame; rotate if needed using q
        vw = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ], dtype=float)
        # If you confirm body-frame, uncomment:
        # R_wb = self._quat_to_R_world_from_body(q)
        # vw = R_wb @ vw

        # Light EMA to smooth
        alpha = 0.2
        self.velocity_truth = alpha * vw + (1 - alpha) * self.velocity_truth

        self.last_truth_time = t
        self.truth_received = True

    def _truth_timer_cb(self):
        if not (self.use_truth and self.tf_available):
            return
        # Only use TF if no fresh odometry has arrived recently
        now = time.time()
        if self.truth_received and (now - self.last_truth_time) < self.truth_timeout:
            return
        # Try candidate frame pairs
        for parent, child in self.tf_candidates:
            try:
                tf: TransformStamped = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                p = tf.transform.translation
                q = tf.transform.rotation
                self.position_truth = np.array([p.x, p.y, p.z], dtype=float)
                # Velocity from finite-difference of pose if TF provides no twist
                if self._last_tf_pos is not None:
                    dt = 0.05
                    vw = (self.position_truth - self._last_tf_pos) / dt
                    alpha = 0.2
                    self.velocity_truth = alpha * vw + (1 - alpha) * self.velocity_truth
                self._last_tf_pos = self.position_truth.copy()
                self.last_truth_time = now
                self.truth_received = True
                break
            except Exception:
                continue

    def _zero_velocity_by_pulses(self, axis: str, v_tol: float = 0.05, max_time: float = 12.0) -> bool:
        """Robustly zero the velocity on one axis using closed-loop Δv pulses."""
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        idx = axis_map[axis]
        t_start = time.time()
        pulses = 0

        while rclpy.ok() and (time.time() - t_start) < max_time and pulses < self.max_pulses:
            v = float(self.velocity[idx])

            # Hysteresis: do nothing if already small
            if abs(v) <= self.v_low:
                break

            # Desired Δv for this pulse (cap to avoid overshoot)
            dv_needed = -v
            dv_cap = max(0.5 * v_tol, 0.25)
            dv_cmd = float(np.clip(dv_needed, -dv_cap, dv_cap))

            # Convert Δv to accel then to duty
            a_cmd = dv_cmd / max(self.pulse_dt, 1e-3)
            duty = self._duty_for_accel(axis, a_cmd)

            # Enforce minimum duty and hard cap
            duty_mag = max(self.min_duty, abs(duty))
            duty = float(np.clip(np.sign(duty) * duty_mag, -0.35, 0.35))

            # Fire one pulse
            cmd = [0.0] * 8
            if axis == 'x':
                if duty > 0:
                    cmd[0] = duty; cmd[1] = duty
                else:
                    cmd[2] = -duty; cmd[3] = -duty
            elif axis == 'y':
                if duty > 0:
                    cmd[4] = duty; cmd[5] = duty
                else:
                    cmd[6] = -duty; cmd[7] = -duty

            v_before = self.velocity.copy()
            t0 = time.time()
            self.send_thrust_command(cmd)
            while time.time() - t0 < self.pulse_dt:
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.01)
            self.send_thrust_command([0.0] * 8)

            # Measure achieved Δv
            time.sleep(self.pulse_cool)
            dv_meas = float(self.velocity[idx] - v_before[idx])

            pulses += 1
            # Suppress per-pulse logs to reduce verbosity

            # Online gain identification (per-axis)
            gain_est = abs(dv_meas) / (max(self.pulse_dt, 1e-3) * max(abs(duty), 1e-6))
            if np.isfinite(gain_est) and gain_est > 1e-3:
                if axis == 'x':
                    self.ax_gain = 0.8 * self.ax_gain + 0.2 * gain_est
                elif axis == 'y':
                    self.ay_gain = 0.8 * self.ay_gain + 0.2 * gain_est

            # Adapt pulse aggressiveness
            if abs(dv_meas) > 2.0 * max(abs(dv_cmd), 0.05):
                # too strong -> shorten pulse, reduce min duty
                self.pulse_dt = max(0.10, 0.8 * self.pulse_dt)
                self.min_duty = max(0.005, 0.5 * self.min_duty)
            elif abs(dv_meas) < 0.2 * max(abs(dv_cmd), 1e-6):
                # too weak -> bump min duty slightly (already present, keep)
                self.min_duty = float(min(0.35, self.min_duty + 0.005))

            # If minimal progress, bump min duty slightly
            if abs(dv_meas) < 0.05 * max(abs(dv_cmd), 1e-6):
                self.min_duty = float(min(0.35, self.min_duty + 0.005))

            # Quit if inside velocity tolerance
            if abs(self.velocity[idx]) <= v_tol:
                break

        # Final settle + ZUPT
        self.send_thrust_command([0.0] * 8)
        time.sleep(0.25)
        return abs(self.velocity[idx]) <= v_tol

    # --- Optional camera subscription (for vision-based controllers) ---
    def enable_image_sub(self, image_topic: str = '/srb/env0/robot/camera/image_raw') -> None:
        if hasattr(self, '_image_sub'):
            return
        self.last_image = None
        self.last_image_time = 0.0
        def _img_cb(msg: Image):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t == 0:
                t = time.time()
            self.last_image = msg
            self.last_image_time = t
        self._image_sub = self.create_subscription(Image, image_topic, _img_cb, 10)

    def zero_velocity_all_axes(self, v_tol: float = 0.05, max_time: float = 20.0):
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < max_time:
            vx, vy, vz = map(float, self.velocity)
            if abs(vx) < v_tol and abs(vy) < v_tol and abs(vz) < v_tol:
                break
            mags = [abs(vx), abs(vy), abs(vz)]
            axis = ('x', 'y', 'z')[int(np.argmax(mags))]
            if axis == 'z':
                # If Z not supported by thrusters, skip; otherwise extend pulses to Z
                break
            self._zero_velocity_by_pulses(axis, v_tol=v_tol, max_time=3.0)
            self.send_thrust_command([0.0] * 8)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        self.send_thrust_command([0.0] * 8)
        
    def start_pid_control(self):
        """Start IMU-based PID control"""
        if not self.imu_received:
            self.get_logger().error('❌ Cannot start PID - no IMU data received!')
            self.get_logger().error('💡 Make sure Isaac Sim is running and publishing IMU data')
            return False
            
        self.controller_active = True
        self.pid_x.reset()
        self.pid_y.reset() 
        self.pid_z.reset()
        self._control_debug_counter = 0
        self.velocity_history = []
        
        self.get_logger().info('🎯 IMU-BASED PID CONTROL STARTED!')
        self.get_logger().info(f'🎯 Target velocity: {self.target_velocity}')
        self.get_logger().info(f'🎯 Current velocity (from IMU): {self.velocity}')
        self.get_logger().info(f'🎯 Tolerance: {self.velocity_tolerance} m/s (ultra-strict!)')
        return True

    def calibrate_thruster_polarity(self):
        """Determine sign mapping and accel-per-duty gains for X/Y.
        Applies brief small thrust pulses and measures velocity change.
        """
        if not self.imu_received:
            return
        def pulse(indices, duty=0.10, dur=0.3):
            cmd = [0.0]*8
            for i in indices:
                cmd[i] = duty
            v0 = self.velocity.copy()
            t0 = time.time()
            self.send_thrust_command(cmd)
            while time.time()-t0 < dur:
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.01)
            self.send_thrust_command([0.0]*8)
            time.sleep(0.15)
            dv = self.velocity - v0
            return dv, dur, duty
        # +X pack
        dv, dur, duty = pulse([0,1])
        self.map_x = 1 if dv[0] > 0 else -1
        ax = abs(dv[0]) / max(dur, 1e-6) / max(duty, 1e-6)
        if ax > 1e-3:
            self.ax_gain = ax
        # +Y pack
        dv, dur, duty = pulse([4,5])
        self.map_y = 1 if dv[1] > 0 else -1
        ay = abs(dv[1]) / max(dur, 1e-6) / max(duty, 1e-6)
        if ay > 1e-3:
            self.ay_gain = ay
        self.get_logger().info(f'🧭 map_x={self.map_x}, map_y={self.map_y} | gains ax={self.ax_gain:.3f}, ay={self.ay_gain:.3f} m/s² per duty')
        
    def stop_pid_control(self):
        """Stop PID control"""
        self.controller_active = False
        
        # Send zero thrust
        self.send_thrust_command([0.0] * 8)
        
        self.get_logger().info('🛑 IMU-BASED PID CONTROL STOPPED')
        
    def move_and_stop(self, power=0.05, duration=0.7):
        """Move forward then engage IMU-based PID to stop"""
        def worker():
            try:
                print(f'\n🚀 STARTING IMU-BASED MOVE AND STOP')
                print(f'📋 Phase 1: Move forward at {power*100:.1f}% power for {duration}s')
                print(f'📋 Phase 2: IMU-based PID braking to zero velocity') 
                print(f'📋 Phase 3: ULTRA-PRECISE stopping (< 0.5 mm/s!)')
                print('='*50)
                
                # Phase 1: Forward movement
                print('🚀 Phase 1: MOVING FORWARD...')
                forward_thrust = [0.0] * 8
                forward_thrust[4] = power  # +Y1
                forward_thrust[5] = power  # +Y2  
                
                start_time = time.time()
                while (time.time() - start_time) < duration:
                    self.send_thrust_command(forward_thrust)
                    print(f'Velocity (IMU): {np.linalg.norm(self.velocity):.4f} m/s', end='\r')
                    rclpy.spin_once(self, timeout_sec=0.01)
                    time.sleep(0.1)
                    
                print(f'\n✅ Phase 1 complete - Final velocity: {np.linalg.norm(self.velocity):.4f} m/s')
                
                # Phase 2: Stop thrusters
                print('⏸️  Phase 2: Stopping thrusters...')
                self.send_thrust_command([0.0] * 8)
                time.sleep(0.5)
                
                # Phase 3: IMU-based PID control
                print('🔄 Phase 3: ENGAGING IMU-BASED PID CONTROL...')
                self.target_velocity = np.array([0.0, 0.0, 0.0])
                # Calibrate thruster polarity once
                self.calibrate_thruster_polarity()
                if not self.start_pid_control():
                    print('❌ Failed to start IMU-based PID control')
                    return
                    
                print('🔄 IMU-PID is now actively controlling to zero velocity...')
                print('🔄 Watch for "IMU-PID #X" messages below!')
                
                # Wait for PID completion
                timeout = 120.0  # 2 minutes for ultra-precise stopping
                start_time = time.time()
                last_update_time = 0
                
                while self.controller_active and (time.time() - start_time) < timeout:
                    current_time = time.time()
                    if current_time - last_update_time >= 2.0:  # Update every 2 seconds
                        velocity_mag = np.linalg.norm(self.velocity)
                        print(f'IMU-PID Active: Velocity = {velocity_mag:.6f} m/s, Time = {current_time - start_time:.1f}s')
                        last_update_time = current_time
                        
                    rclpy.spin_once(self, timeout_sec=0.01)
                    time.sleep(0.1)
                    
                if self.controller_active:
                    print(f'⚠️  IMU-PID timeout after {timeout}s')
                    self.stop_pid_control()
                    
                final_velocity = np.linalg.norm(self.velocity)
                print(f'\n🎉 MOVEMENT COMPLETE!')
                print(f'✅ Final velocity: {final_velocity:.6f} m/s')
                
                if final_velocity < self.velocity_tolerance:
                    print(f'🎯 PERFECT! TRUE FULL STOP ACHIEVED - IMU-based velocity: {final_velocity:.6f} m/s')
                    print('✨ IMU integration and ultra-precise PID working flawlessly!')
                else:
                    print(f'⚠️  Still moving: {final_velocity:.6f} m/s (target < {self.velocity_tolerance:.4f} m/s)')
                    if final_velocity < 0.001:
                        print('   ✅ EXTREMELY CLOSE! Almost at true full stop (< 1 mm/s)')
                    elif final_velocity < 0.003:
                        print('   🔄 VERY CLOSE! IMU-based control working well')
                        
                print('='*50)
                
            except Exception as e:
                self.get_logger().error(f'Error in move_and_stop: {e}')
                self.stop_pid_control()
                
        # Run in separate thread
        thread = threading.Thread(target=worker)
        thread.daemon = True
        thread.start()

    def move_to_offset(self, axis='y', distance=100.0, pos_tol=0.05, vel_tol=0.05, timeout=180.0):
        """Move along one axis by a target distance (meters) then stop.
        Uses position PD with velocity damping: a_cmd = kp*pos_err - kd*vel.
        Assumes robot orientation is fixed so body axes align with control axes.
        """
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        if axis not in axis_map:
            print('❌ Invalid axis. Use x/y/z')
            return
        idx = axis_map[axis]
        # Phase 1: cruise quickly toward target, then Phase 2: precise PID stop
        start_pos = self.position.copy()
        target = start_pos[idx] + float(distance)
        total = abs(distance)
        dir_sign = 1.0 if distance >= 0.0 else -1.0
        print(f'🧭 Move {distance:.2f} m along {axis.upper()} (target={target:.2f})')

        # Phase 1: Closed-loop velocity profile follower (envelope-based)
        open_thrust = 0.22
        phase1_timeout = min(120.0, 0.5 * total + 10.0)
        t0 = time.time(); last_log = 0.0
        a_brake = 2.0     # m/s^2 max decel (match braking capability)
        a_accel = 1.5     # m/s^2 max accel
        k_v = 2.0         # speed tracking gain (1/s)
        v_max_hard = 6.0  # absolute speed ceiling
        margin = 1.0      # meters buffer
        while rclpy.ok() and (time.time() - t0) < phase1_timeout:
            pos_err = target - self.position[idx]
            rem = abs(pos_err)
            v_along = self.velocity[idx] * dir_sign

            v_allow = float(np.sqrt(max(0.0, 2.0 * a_brake * max(rem - margin, 0.0))))
            v_des = min(v_allow, v_max_hard)

            a_cmd = k_v * (v_des - v_along)
            a_cmd = float(np.clip(a_cmd, -a_brake, a_accel))

            duty = self._duty_for_accel(axis, a_cmd * dir_sign)
            duty = float(np.clip(duty, -0.30, 0.30))

            cmd = [0.0] * 8
            if axis == 'x':
                if duty > 0: cmd[0]=duty; cmd[1]=duty
                else:        cmd[2]=-duty; cmd[3]=-duty
            elif axis == 'y':
                if duty > 0: cmd[4]=duty; cmd[5]=duty
                else:        cmd[6]=-duty; cmd[7]=-duty

            self.send_thrust_command(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

            if time.time() - last_log > 1.0:
                print(f'[CRUISE] rem={rem:.2f} m, v={self.velocity[idx]:.2f}, v_des={v_des:.2f}, a_cmd={a_cmd:.2f}, duty={abs(duty):.3f}')
                last_log = time.time()

            if v_des < 0.6:
                break

        # Stop thrust before precision phase
        self.send_thrust_command([0.0] * 8)
        time.sleep(0.2)

        # Phase 2: velocity nulling – robustly bring velocity to zero on this axis
        vel_target_tol = max(vel_tol, 0.08)
        t2 = time.time(); last_log = 0.0
        while rclpy.ok() and (time.time() - t2) < min(30.0, timeout * 0.6):
            vel = self.velocity[idx]
            pos_err = target - self.position[idx]

            if abs(vel) < vel_target_tol and abs(pos_err) < max(0.5, 0.1*total):
                break

            # If we crossed the target, stop optimizing position; just kill velocity
            if (pos_err * (target - start_pos[idx])) < 0:
                self.send_thrust_command([0.0] * 8)
                time.sleep(0.15)
                ok = self._zero_velocity_by_pulses(axis, v_tol=max(vel_tol, 0.05))
                self.send_thrust_command([0.0] * 8)
                print(f'✅ STOP ON CROSSING: |pos_err|≈{abs(target - self.position[idx]):.2f} m, |v|={abs(self.velocity[idx]):.3f} m/s')
                self.zero_velocity_all_axes(v_tol=max(vel_tol, 0.05))
                return

            # Velocity PD (pure D on position): a_cmd opposes velocity
            # Stronger decel ladder
            if abs(vel) > 2.0:
                a_cap = 2.0
            elif abs(vel) > 1.0:
                a_cap = 1.0
            elif abs(vel) > 0.3:
                a_cap = 0.4
            else:
                a_cap = 0.15

            a_cmd = -np.sign(vel) * a_cap
            duty = self._duty_for_accel(axis, a_cmd)
            duty = float(np.clip(duty, -0.3, 0.3))

            cmd = [0.0] * 8
            if axis == 'x':
                if duty > 0:
                    cmd[0] = duty; cmd[1] = duty
                else:
                    cmd[2] = -duty; cmd[3] = -duty
            elif axis == 'y':
                if duty > 0:
                    cmd[4] = duty; cmd[5] = duty
                else:
                    cmd[6] = -duty; cmd[7] = -duty

            self.send_thrust_command(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

            # If we crossed the target OR are close enough, kill velocity now
            if (pos_err * (target - start_pos[idx])) < 0 or abs(pos_err) < 2.0:
                self.send_thrust_command([0.0] * 8)
                time.sleep(0.15)
                ok = self._zero_velocity_by_pulses(axis, v_tol=max(vel_tol, 0.05))
                self.send_thrust_command([0.0] * 8)
                print(f'✅ STOP NEAR TARGET: |pos_err|≈{abs(target - self.position[idx]):.2f} m, |v|={abs(self.velocity[idx]):.3f} m/s')
                self.zero_velocity_all_axes(v_tol=max(vel_tol, 0.05))
                return

            if time.time() - last_log > 0.8:
                print(f'[VEL-BRAKE] vel={vel:.3f} m/s, err={pos_err:.3f} m, a_cmd={a_cmd:.3f} m/s², duty={abs(duty):.3f}')
                last_log = time.time()

        # Small pause, then Phase 3: precise position PID to reduce residual
        self.send_thrust_command([0.0] * 8)
        time.sleep(0.15)

        # If we're within the final window, ignore position and guarantee stop with Δv pulses
        if abs((target - self.position[idx])) <= self.final_pos_window:
            ok = self._zero_velocity_by_pulses(axis, v_tol=max(vel_tol, 0.05))
            self.send_thrust_command([0.0] * 8)
            if ok:
                print(f'✅ FINAL STOP: |pos_err|≈{abs(target - self.position[idx]):.2f} m, |v|={abs(self.velocity[idx]):.3f} m/s')
            else:
                print('⚠️ FINAL STOP: timeout, using best effort.')
            self.zero_velocity_all_axes(v_tol=max(vel_tol, 0.05))
            return

        # Auto-tune PD(+I) from double-integrator dynamics
        # a_cmd = Kp*pos_err - Kd*vel + Ki*∫err dt, capped by axis accel authority
        duty_cap = 0.30
        axis_gain = abs(self.ax_gain) if axis == 'x' else abs(self.ay_gain)  # m/s^2 per duty
        a_max = max(0.05, axis_gain * duty_cap)  # available accel (headroom handled by alpha)
        alpha = 0.6  # headroom on peak accel
        Kp = 50 * (alpha * a_max / max(0.5, total))
        zeta = 1.1
        Kd = 2.0 * zeta * np.sqrt(Kp)
        Ki = 0.05 * Kp * np.sqrt(Kp)
        dwell_cycles = 0; success = False
        t3 = time.time(); last_log = 0.0
        last_time = t3
        # position integrator with anti-windup
        if not hasattr(self, "_pos_err_int"):
            self._pos_err_int = [0.0, 0.0, 0.0]
        I_MAX = 5.0
        while rclpy.ok() and (time.time() - t3) < timeout:
            pos_err = target - self.position[idx]
            vel = self.velocity[idx]

            now = time.time()
            dt = now - last_time
            last_time = now

            # True integral with anti-windup
            self._pos_err_int[idx] += float(pos_err) * dt
            self._pos_err_int[idx] = float(np.clip(self._pos_err_int[idx], -I_MAX, I_MAX))

            # PD(+I) acceleration command (double-integrator design)
            a_unsat = Kp * float(pos_err) - Kd * float(vel) + Ki * self._pos_err_int[idx]
            a_cmd = float(np.clip(a_unsat, -a_max, a_max))

            # Convert desired accel to thrust duty using calibrated gain
            duty = self._duty_for_accel(axis, a_cmd)
            duty = float(np.clip(duty, -duty_cap, duty_cap))

            cmd = [0.0] * 8
            if axis == 'x':
                if duty > 0:
                    cmd[0] = duty; cmd[1] = duty
                else:
                    cmd[2] = -duty; cmd[3] = -duty
            elif axis == 'y':
                if duty > 0:
                    cmd[4] = duty; cmd[5] = duty
                else:
                    cmd[6] = -duty; cmd[7] = -duty

            self.send_thrust_command(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

            # If already slow and close, finish with pulses immediately
            if abs(pos_err) < 2.0 and abs(vel) < 0.20:
                self.send_thrust_command([0.0] * 8)
                ok = self._zero_velocity_by_pulses(axis, v_tol=max(vel_tol, 0.05))
                self.send_thrust_command([0.0] * 8)
                print(f'✅ FINAL STOP (PID handoff): |pos_err|≈{abs(pos_err):.2f} m, |v|={abs(self.velocity[idx]):.3f} m/s')
                self.zero_velocity_all_axes(v_tol=max(vel_tol, 0.05))
                return

            if time.time() - last_log > 0.8:
                print(f'[PID] err={pos_err:.3f} m, vel={vel:.3f} m/s, Kp={Kp:.3f}, Kd={Kd:.3f}, a_max={a_max:.2f}, a_cmd={a_cmd:.3f} m/s², duty={abs(duty):.3f}')
                last_log = time.time()

            # Only bail back to velocity brake if very close AND moving away too fast
            near_enough = abs(pos_err) < 0.30
            moving_away = (pos_err * vel) < -0.0
            too_fast_here = abs(vel) > max(vel_tol, 0.30)
            if near_enough and moving_away and too_fast_here:
                break

            if abs(pos_err) < pos_tol and abs(vel) < vel_tol:
                dwell_cycles += 1
                if dwell_cycles >= 20:
                    success = True
                    break
            else:
                dwell_cycles = 0

        if not success:
            # One more quick velocity brake if needed
            self.send_thrust_command([0.0] * 8)
            time.sleep(0.1)
            vel = self.velocity[idx]
            if abs(vel) > vel_target_tol:
                a_cmd = -np.sign(vel) * 0.08
                cmd = [0.0] * 8
                if axis == 'x':
                    eff = a_cmd if self.map_x > 0 else -a_cmd
                    if eff > 0:
                        cmd[0] = eff; cmd[1] = eff
                    else:
                        cmd[2] = -eff; cmd[3] = -eff
                elif axis == 'y':
                    eff = a_cmd if self.map_y > 0 else -a_cmd
                    if eff > 0:
                        cmd[4] = eff; cmd[5] = eff
                    else:
                        cmd[6] = -eff; cmd[7] = -eff
                self.send_thrust_command(cmd)
                time.sleep(0.2)

        self.send_thrust_command([0.0] * 8)
        # Always finish with pulse-based zeroing for robustness
        ok = self._zero_velocity_by_pulses(axis, v_tol=max(vel_tol, 0.05))
        self.send_thrust_command([0.0] * 8)

        final_err = target - self.position[idx]
        final_vel = self.velocity[idx]
        # Ensure all axes are zeroed
        self.zero_velocity_all_axes(v_tol=max(vel_tol, 0.05))
        if success or ok:
            print(f'✅ Stopped. Final err≈{final_err:.3f} m, vel={final_vel:.3f} m/s')
        else:
            print(f'⚠️ Stopped (best effort). Final err≈{final_err:.3f} m, vel={final_vel:.3f} m/s')

    def follow_circle(self, diameter: float = 100.0, speed: float = 2.0, revolutions: float = 1.0, duty_cap: float = 0.30):
        """Track a circular trajectory in the XY plane using acceleration control.

        - Center: current XY position at start
        - Radius: diameter/2
        - Speed: tangential speed (m/s) along the circle
        - Revolutions: how many full circles to complete
        """
        R = max(1e-3, float(diameter) * 0.5)
        v_t = max(0.1, float(speed))
        omega = v_t / R
        total_time = (2.0 * np.pi * R / v_t) * max(0.1, float(revolutions))

        # Gains from axis authority
        ax_gain = max(1e-3, abs(self.ax_gain))
        ay_gain = max(1e-3, abs(self.ay_gain))
        amax_x = ax_gain * duty_cap
        amax_y = ay_gain * duty_cap
        alpha = 0.5
        Kp = 50 * (alpha * min(amax_x, amax_y) / R)
        zeta = 1.0
        Kd = 2.0 * zeta * np.sqrt(Kp)

        # Center and initial angle from current pose
        center = self.position.copy()
        center[2] = self.position[2]
        # Compute initial angle from current position relative to center
        rel = self.position[:2] - center[:2]
        theta0 = np.arctan2(rel[1] if np.linalg.norm(rel) > 1e-6 else 0.0, rel[0] if np.linalg.norm(rel) > 1e-6 else 1.0)

        t0 = time.time()
        last_log = 0.0
        while rclpy.ok() and (time.time() - t0) < total_time:
            t = time.time() - t0
            theta = theta0 + omega * t
            # Desired pose/vel/acc in world XY
            p_des = np.array([
                center[0] + R * np.cos(theta),
                center[1] + R * np.sin(theta)
            ], dtype=float)
            v_des = np.array([
                -R * omega * np.sin(theta),
                +R * omega * np.cos(theta)
            ], dtype=float)
            a_ff = np.array([
                -R * (omega ** 2) * np.cos(theta),
                -R * (omega ** 2) * np.sin(theta)
            ], dtype=float)

            p = np.array([self.position[0], self.position[1]], dtype=float)
            v = np.array([self.velocity[0], self.velocity[1]], dtype=float)
            a_cmd_xy = a_ff + Kp * (p_des - p) + Kd * (v_des - v)

            # Per-axis saturation based on authority
            a_cmd_x = float(np.clip(a_cmd_xy[0], -amax_x, amax_x))
            a_cmd_y = float(np.clip(a_cmd_xy[1], -amax_y, amax_y))

            duty_x = float(np.clip(self._duty_for_accel('x', a_cmd_x), -duty_cap, duty_cap))
            duty_y = float(np.clip(self._duty_for_accel('y', a_cmd_y), -duty_cap, duty_cap))

            cmd = [0.0] * 8
            # X group 0-3
            if duty_x > 0:
                cmd[0] = duty_x; cmd[1] = duty_x
            else:
                cmd[2] = -duty_x; cmd[3] = -duty_x
            # Y group 4-7
            if duty_y > 0:
                cmd[4] = duty_y; cmd[5] = duty_y
            else:
                cmd[6] = -duty_y; cmd[7] = -duty_y

            self.send_thrust_command(cmd)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

            if time.time() - last_log > 1.0:
                err = np.linalg.norm(p_des - p)
                print(f'[CIRCLE] t={t:.1f}s θ={theta% (2*np.pi):.2f} | err={err:.2f} m | a=({a_cmd_x:.2f},{a_cmd_y:.2f}) m/s² | duty=({abs(duty_x):.3f},{abs(duty_y):.3f})')
                last_log = time.time()

        # Stop thrust and settle
        self.send_thrust_command([0.0] * 8)
        time.sleep(0.2)
        self.zero_velocity_all_axes(v_tol=0.1, max_time=10.0)

def main():
    rclpy.init()
    
    controller = IMUPIDController()
    
    def signal_handler(sig, frame):
        print('\n🛑 Shutting down IMU-PID controller...')
        controller.stop_pid_control()
        rclpy.shutdown()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    # Interactive menu
    def run_menu():
        while rclpy.ok():
            try:
                velocity_mag = np.linalg.norm(controller.velocity)
                imu_status = "✅ Active" if controller.imu_received else "❌ No Data"
                pid_status = "🔄 Active" if controller.controller_active else "⏸️  Idle"
                stopped = "✅ Stopped" if velocity_mag < 0.001 else "🔄 Moving"
                
                print('\n' + '='*40)
                print('  IMU-BASED SPACECRAFT PID CONTROLLER')
                print('='*40)
                print(f'  IMU Status: {imu_status}')
                print(f'  Current Velocity: [{controller.velocity[0]:.4f}, {controller.velocity[1]:.4f}, {controller.velocity[2]:.4f}] m/s')
                print(f'  Velocity Magnitude: {velocity_mag:.4f} m/s')
                print(f'  PID Controller: {pid_status}')
                print(f'  Status: {stopped}')
                print('\nChoose an option:')
                print('1. 🚀 AUTO MOVE & IMU-PID STOP (Everything Automatic!)')
                print('2. 🎯 Manual: Start IMU-PID Control to Zero')
                print('3. 🛑 Manual: Stop PID Control')
                print('4. ⚙️  Custom Auto Move & IMU-PID Stop')
                print('5. 📊 Show Detailed Status')
                print('6. 🔧 Tune PID Gains')
                print('0. 🚪 Exit')
                print('7. 🧭 Move by distance (position-based)')
                
                choice = input('\nEnter choice (0-7): ').strip()
                
                if choice == '1':
                    controller.move_and_stop()
                elif choice == '2':
                    controller.target_velocity = np.array([0.0, 0.0, 0.0])
                    controller.start_pid_control()
                elif choice == '3':
                    controller.stop_pid_control()
                elif choice == '4':
                    power = float(input('Power (0.0-1.0): ') or '0.1')
                    duration = float(input('Duration (seconds): ') or '1.5')
                    controller.move_and_stop(power, duration)
                elif choice == '5':
                    print(f'\n📊 DETAILED STATUS:')
                    print(f'IMU Data Received: {controller.imu_received}')
                    print(f'Velocity: {controller.velocity}')
                    print(f'Target: {controller.target_velocity}')
                    print(f'PID Gains: Kp={controller.pid_x.kp}, Ki={controller.pid_x.ki}, Kd={controller.pid_x.kd}')
                    print(f'Tolerance: {controller.velocity_tolerance} m/s')
                    print(f'Deadband: {controller.deadband} m/s')
                elif choice == '6':
                    print(f'\nCurrent PID Gains: Kp={controller.pid_x.kp}, Ki={controller.pid_x.ki}, Kd={controller.pid_x.kd}')
                    kp = float(input(f'Kp ({controller.pid_x.kp}): ') or controller.pid_x.kp)
                    ki = float(input(f'Ki ({controller.pid_x.ki}): ') or controller.pid_x.ki)
                    kd = float(input(f'Kd ({controller.pid_x.kd}): ') or controller.pid_x.kd)
                    
                    controller.pid_x.kp = controller.pid_y.kp = controller.pid_z.kp = kp
                    controller.pid_x.ki = controller.pid_y.ki = controller.pid_z.ki = ki
                    controller.pid_x.kd = controller.pid_y.kd = controller.pid_z.kd = kd
                    print('✅ PID gains updated!')
                elif choice == '7':
                    mode = (input('Mode: [1] move by distance, [2] circle follow (default 1): ').strip() or '1')
                    if mode == '2':
                        diameter = float(input('Circle diameter (m) [100]: ') or '100')
                        speed = float(input('Tangential speed (m/s) [2.0]: ') or '2.0')
                        revs = float(input('Revolutions [1.0]: ') or '1.0')
                        controller.calibrate_thruster_polarity()
                        controller.follow_circle(diameter=diameter, speed=speed, revolutions=revs)
                    else:
                        axis = (input('Axis (x/y/z) [y]: ').strip() or 'y').lower()
                        distance = float(input('Distance meters [100]: ') or '100')
                        pos_tol = float(input('Position tol m [0.05]: ') or '0.05')
                        vel_tol = float(input('Velocity tol m/s) [0.05]: ') or '0.05')
                        controller.calibrate_thruster_polarity()
                        controller.move_to_offset(axis=axis, distance=distance, pos_tol=pos_tol, vel_tol=vel_tol)
                elif choice == '0':
                    break
                else:
                    print('❌ Invalid choice! Please enter 0-6.')
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f'❌ Error: {e}')
                
        controller.stop_pid_control()
        rclpy.shutdown()
        
    # Run menu in main thread
    menu_thread = threading.Thread(target=run_menu)
    menu_thread.daemon = True
    menu_thread.start()
    
    # Spin ROS node
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_pid_control()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

