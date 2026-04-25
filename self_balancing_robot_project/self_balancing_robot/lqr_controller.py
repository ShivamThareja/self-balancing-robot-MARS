import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
from scipy.linalg import solve_continuous_are


class LQRController(Node):
    def __init__(self):
        super().__init__('lqr_controller')

        # ── Robot Physical Parameters (must match URDF) ──
        self.M = 2.0        # Body mass (kg)
        self.m = 0.3        # Each wheel mass (kg)
        self.L = 0.25       # Distance from axle to center of mass (m)
        self.I = 0.05       # Body moment of inertia (kg.m^2)
        self.g = 9.81       # Gravity (m/s^2)
        self.r = 0.06       # Wheel radius (m)

        # ── Linearized State Space Model ──
        # State Vector: [theta, theta_dot, x, x_dot]
        # Linearized around theta = 0 (upright position)
        Mt = self.M + 2 * self.m    # Total mass
        It = self.I + self.M * self.L**2
        det = It * Mt - (self.M * self.L)**2

        # A matrix: describes system dynamics
        a21 = (Mt * self.M * self.g * self.L) / det
        a41 = -(self.M**2 * self.L**2 * self.g) / det

        # B matrix: describes control input effect
        b21 = -(self.M * self.L) / det
        b41 = It / det

        self.A = np.array([
            [0, 1, 0, 0],
            [a21, 0, 0, 0],
            [0, 0, 0, 1],
            [a41, 0, 0, 0]
        ])
        self.B = np.array([[0], [b21], [0], [b41]])

        # ── LQR Cost Matrices ──
        # Q: penalizes state errors [angle, angular_vel, position, velocity]
        # R: penalizes control effort (motor commands)
        self.Q = np.diag([1000.0, 100.0, 300.0, 150.0])
        self.R = np.array([[0.01]])

        # ── Compute Optimal LQR Gain Matrix K ──
        self.K = self.compute_lqr_gain()
        self.Kp_yaw = 2.5
        self.get_logger().info(f'NEW GAINS LOADED. K: {self.K}')

        # ── State Variables ──
        self.theta = 0.0            # Tilt angle (rad)
        self.theta_dot = 0.0        # Angular velocity (rad/s)
        self.x = 0.0                # Position (m)
        self.x_dot = 0.0            # Linear velocity (m/s)
        self.yaw_rate = 0.0         # Yaw rate (rad/s)

        # ── Target References ──
        self.target_x = 0.0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.last_time = self.get_clock().now()

        # ── ROS 2 Interface ──
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.vel_sub = self.create_subscription(
            Twist, '/target_vel', self.vel_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.get_logger().info('*** AGGRESSIVE MOTION CONTROLLER STARTED ***')

    def compute_lqr_gain(self):
        """Solve Continuous Algebraic Riccati Equation to get optimal K"""
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        return np.linalg.inv(self.R) @ self.B.T @ P

    def vel_callback(self, msg):
        """Receives keyboard commands from teleop_key.py"""
        self.target_linear_vel = msg.linear.x
        self.target_angular_vel = msg.angular.z
        self.get_logger().info(
            f'RECEIVED COMMAND: L={self.target_linear_vel} A={self.target_angular_vel}')

    def odom_callback(self, msg):
        """Gets robot position and velocity from odometry"""
        self.x = msg.pose.pose.position.x
        self.x_dot = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        """Main control loop - runs at 200Hz"""
        # 1. Update timing
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0:
            return

        # 2. Extract state from IMU
        self.theta = self.quaternion_to_pitch(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.theta_dot = msg.angular_velocity.y
        self.yaw_rate = msg.angular_velocity.z

        # 3. Safety check - stop if robot has fallen
        if abs(math.degrees(self.theta)) > 45.0:
            self.target_x = self.x
            self.cmd_pub.publish(Twist())
            self.get_logger().warn(
                f'Robot fallen! Angle: {math.degrees(self.theta):.2f}°')
            return

        # 4. Update target position using velocity integration
        self.target_x += self.target_linear_vel * dt

        # 5. Compute state error vector
        state_error = np.array([
            self.theta,                             # Angle error
            self.theta_dot,                         # Angular velocity error
            self.x - self.target_x,                 # Position error
            self.x_dot - self.target_linear_vel     # Velocity error
        ])

        # 6. LQR Control Law: u = -K * state_error
        u_balance = -float(self.K @ state_error)
        u_balance = np.clip(u_balance, -30.0, 30.0)

        # 7. Yaw control (turning)
        u_yaw = self.Kp_yaw * (self.target_angular_vel - self.yaw_rate)
        u_yaw = np.clip(u_yaw, -10.0, 10.0)

        # 8. Publish command to wheels
        cmd = Twist()
        cmd.linear.x = u_balance
        cmd.angular.z = u_yaw
        self.cmd_pub.publish(cmd)

        # 9. Diagnostic logging every 0.2 seconds
        if self.get_clock().now().nanoseconds % 200000000 < 10000000:
            self.get_logger().info(
                f'Angle: {math.degrees(self.theta):.1f}° | '
                f'Pos Err: {self.x - self.target_x:.3f}m | '
                f'Cmd: {u_balance:.2f}'
            )

    def quaternion_to_pitch(self, x, y, z, w):
        """Convert quaternion orientation to pitch angle (tilt)"""
        sinp = 2.0 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)
        return math.asin(sinp)


def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
