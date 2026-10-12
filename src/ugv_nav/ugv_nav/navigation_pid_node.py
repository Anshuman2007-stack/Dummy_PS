import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def get_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PID:
    def __init__(self, kp, ki, kd, max_out, max_i_out):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_out, self.max_i_out = max_out, max_i_out
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev = None

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
        if self.prev is None:
            self.prev = error
        if error * self.prev < 0:                     # crossed target
            self.integral = 0.0
        p = self.kp * error
        if abs(p) < self.max_out:                     # no windup while saturated
            self.integral += error * dt
        if self.ki > 0:
            lim = self.max_i_out / self.ki
            self.integral = max(-lim, min(lim, self.integral))
        d = self.kd * (error - self.prev) / dt
        self.prev = error
        out = p + self.ki * self.integral + d
        return max(-self.max_out, min(self.max_out, out))


class Controller(Node):
    ROTATE, DRIVE, ALIGN, DONE = 'ROTATE', 'DRIVE', 'ALIGN', 'DONE'

    def __init__(self):
        super().__init__('navigation_pid_node')
        for name, val in [('max_linear', 0.3), ('max_angular', 1.0),
                          ('goal_tol', 0.10), ('steer_freeze', 0.15),
                          ('rotate_done', 0.08), ('rotate_again', 0.6),
                          ('align_final', False), ('align_tol', 0.08)]:
            self.declare_parameter(name, val)
        g = lambda n: self.get_parameter(n).value
        self.max_lin, self.max_ang = g('max_linear'), g('max_angular')
        self.tol, self.freeze = g('goal_tol'), g('steer_freeze')
        self.rot_done, self.rot_again = g('rotate_done'), g('rotate_again')
        self.align_final, self.align_tol = g('align_final'), g('align_tol')

        #                       Kp   Ki   Kd   max          max_I
        self.lin_pid = PID(0.8, 0.0, 0.1, self.max_lin, 0.05)
        self.ang_pid = PID(2.0, 0.0, 0.2, self.max_ang, 0.2)

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(Odometry, '/ekf/odom', self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.goal = None
        self.goal_yaw = 0.0
        self.pose = None
        self.state = self.DONE
        self.last = None
        self.create_timer(0.05, self.loop)
        self.get_logger().info("PID navigation ready.")

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_yaw = get_yaw(msg.pose.orientation)
        self.set_state(self.ROTATE)
        self.get_logger().info(f"New goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def pose_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, get_yaw(p.orientation))

    def set_state(self, s):
        self.state = s
        self.lin_pid.reset()
        self.ang_pid.reset()

    def finish(self, text):
        self.set_state(self.DONE)
        self.get_logger().info(text)

    def loop(self):
        if self.pose is None:
            return
        now = self.get_clock().now()
        if self.last is None:
            self.last = now
            return
        dt = (now - self.last).nanoseconds / 1e9
        self.last = now

        cmd = Twist()
        if self.goal is None or self.state == self.DONE:
            self.pub.publish(cmd)
            return

        x, y, yaw = self.pose
        dx, dy = self.goal[0] - x, self.goal[1] - y
        dist = math.hypot(dx, dy)
        err = wrap(math.atan2(dy, dx) - yaw)

        if self.state in (self.ROTATE, self.DRIVE) and dist < self.tol:
            if self.align_final:
                self.set_state(self.ALIGN)
            else:
                self.finish("Goal reached.")
            self.pub.publish(cmd)
            return

        if self.state == self.ROTATE:
            if abs(err) < self.rot_done:
                self.set_state(self.DRIVE)
            else:
                cmd.angular.z = self.ang_pid.compute(err, dt)

        elif self.state == self.DRIVE:
            if dist > self.freeze and abs(err) > self.rot_again:
                self.set_state(self.ROTATE)
            elif dist <= self.freeze and abs(err) > math.pi / 2:
                # Slid past the goal: close enough -> stop, otherwise turn around
                if dist < 2 * self.tol:
                    self.finish("Goal reached (passed within 2x tolerance).")
                else:
                    self.set_state(self.ROTATE)
            else:
                cmd.linear.x = self.lin_pid.compute(dist, dt) * max(0.0, math.cos(err))
                if dist > self.freeze:                 # heading is noisy near the goal
                    cmd.angular.z = self.ang_pid.compute(err, dt)

        elif self.state == self.ALIGN:
            e = wrap(self.goal_yaw - yaw)
            if abs(e) < self.align_tol:
                self.finish("Goal reached and aligned.")
            else:
                cmd.angular.z = self.ang_pid.compute(e, dt)

        self.get_logger().info(
            f"{self.state} pos=({x:.2f},{y:.2f}) dist={dist:.2f} "
            f"err={math.degrees(err):.0f}deg cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f})",
            throttle_duration_sec=0.5)
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
