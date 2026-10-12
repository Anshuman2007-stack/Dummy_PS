import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def get_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.declare_parameter('gps_sigma', 0.3)         # m, match your GPS noise
        self.declare_parameter('gps_gate', 9.21)         # chi2, 2 dof, 99%
        self.declare_parameter('gps_init_samples', 20)
        self.declare_parameter('use_imu_yaw', True)

        self.gps_sigma = self.get_parameter('gps_sigma').value
        self.gps_gate = self.get_parameter('gps_gate').value
        self.gps_n = self.get_parameter('gps_init_samples').value
        self.use_imu_yaw = self.get_parameter('use_imu_yaw').value

        # State: [x, y, theta, v, omega]
        self.x = np.zeros((5, 1))
        self.P = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        self.Q = np.diag([1e-3, 1e-3, 1e-3, 0.1, 0.1])    # per second

        self.R_odom = np.diag([0.01, 0.02])               # v, omega
        self.R_gyro = np.array([[0.01]])
        self.R_yaw = np.array([[0.01]])
        self.R_gps = np.eye(2) * self.gps_sigma ** 2

        self.ready = False            # set on first wheel-odom message
        self.start_xy = (0.0, 0.0)    # EKF position when GPS origin was taken
        self.imu_yaw_offset = None
        self.gps_ref = None
        self.gps_samples = []
        self.gps_rejects = 0
        self.last_time = self.get_clock().now()

        self.create_subscription(Odometry, '/odom/wheels', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gnss/fix', self.gps_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Odometry, '/ekf/odom', 10)
        self.create_timer(0.02, self.predict)             # 50 Hz
        self.get_logger().info("EKF started. Keep the robot still until 'GPS origin set'.")

    # ---------- prediction ----------
    def predict(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if not self.ready or dt <= 0.0 or dt > 0.5:
            return

        th, v, w = self.x[2, 0], self.x[3, 0], self.x[4, 0]
        self.x[0, 0] += v * math.cos(th) * dt
        self.x[1, 0] += v * math.sin(th) * dt
        self.x[2, 0] = wrap(th + w * dt)

        F = np.eye(5)
        F[0, 2] = -v * math.sin(th) * dt
        F[0, 3] = math.cos(th) * dt
        F[1, 2] = v * math.cos(th) * dt
        F[1, 3] = math.sin(th) * dt
        F[2, 4] = dt
        self.P = F @ self.P @ F.T + self.Q * dt
        self.publish(now)

    # ---------- generic update ----------
    def update(self, z, H, R, angle_row=None, gate=None):
        y = z - H @ self.x
        if angle_row is not None:
            y[angle_row, 0] = wrap(y[angle_row, 0])
        S = H @ self.P @ H.T + R
        if gate is not None:
            d2 = float(y.T @ np.linalg.inv(S) @ y)
            if d2 > gate:
                return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2, 0] = wrap(self.x[2, 0])
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T      # Joseph form
        return True

    # ---------- measurements ----------
    def odom_cb(self, msg):
        if not self.ready:
            p = msg.pose.pose
            self.x[0, 0] = p.position.x
            self.x[1, 0] = p.position.y
            self.x[2, 0] = get_yaw(p.orientation)
            self.ready = True
            self.get_logger().info(
                f"Start pose ({self.x[0,0]:.2f}, {self.x[1,0]:.2f}, "
                f"{math.degrees(self.x[2,0]):.0f} deg)")
        z = np.array([[msg.twist.twist.linear.x], [msg.twist.twist.angular.z]])
        H = np.zeros((2, 5)); H[0, 3] = 1.0; H[1, 4] = 1.0
        self.update(z, H, self.R_odom)

    def imu_cb(self, msg):
        if not self.ready:
            return
        H = np.zeros((1, 5)); H[0, 4] = 1.0
        self.update(np.array([[msg.angular_velocity.z]]), H, self.R_gyro)

        q = msg.orientation
        if not self.use_imu_yaw or (q.x == q.y == q.z == q.w == 0.0):
            return
        yaw = get_yaw(q)
        if self.imu_yaw_offset is None:
            self.imu_yaw_offset = wrap(self.x[2, 0] - yaw)
            return
        H = np.zeros((1, 5)); H[0, 2] = 1.0
        self.update(np.array([[wrap(yaw + self.imu_yaw_offset)]]), H,
                    self.R_yaw, angle_row=0)

    def gps_cb(self, msg):
        if not self.ready or math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        lat, lon = math.radians(msg.latitude), math.radians(msg.longitude)

        if self.gps_ref is None:                     # average first fixes
            self.gps_samples.append((lat, lon))
            if len(self.gps_samples) >= self.gps_n:
                n = len(self.gps_samples)
                self.gps_ref = (sum(s[0] for s in self.gps_samples) / n,
                                sum(s[1] for s in self.gps_samples) / n)
                self.start_xy = (self.x[0, 0], self.x[1, 0])
                self.get_logger().info("GPS origin set. Ready for goals.")
            return

        R_E = 6378137.0
        east = R_E * (lon - self.gps_ref[1]) * math.cos(self.gps_ref[0])
        north = R_E * (lat - self.gps_ref[0])
        z = np.array([[self.start_xy[0] + east], [self.start_xy[1] + north]])
        H = np.zeros((2, 5)); H[0, 0] = 1.0; H[1, 1] = 1.0

        ok = self.update(z, H, self.R_gps, gate=self.gps_gate)
        if ok:
            self.gps_rejects = 0
        else:
            self.gps_rejects += 1
            if self.gps_rejects > 20:                # filter is lost: trust GPS
                self.update(z, H, self.R_gps)
                self.gps_rejects = 0
                self.get_logger().warn("GPS rejected too long, resynced.")

    # ---------- output ----------
    def publish(self, stamp):
        m = Odometry()
        m.header.stamp = stamp.to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = float(self.x[0, 0])
        m.pose.pose.position.y = float(self.x[1, 0])
        m.pose.pose.orientation.z = math.sin(self.x[2, 0] / 2.0)
        m.pose.pose.orientation.w = math.cos(self.x[2, 0] / 2.0)
        m.twist.twist.linear.x = float(self.x[3, 0])
        m.twist.twist.angular.z = float(self.x[4, 0])
        m.pose.covariance[0] = float(self.P[0, 0])
        m.pose.covariance[7] = float(self.P[1, 1])
        m.pose.covariance[35] = float(self.P[2, 2])
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EKFNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
