"""Simulated ESP32 mission state machine: 'advance D meters' with reactive avoidance."""
import math
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8

from roverto_autonomy.calibration import load_calibration

STANDBY, EXECUTING, AVOIDING, GOAL_REACHED = 0, 1, 2, 3
CLEAR, AVOID_LEFT, AVOID_RIGHT, STOP, HOLD_HEADING = 0, 1, 2, 3, 4


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    """Extract yaw from a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimEsp32Emulator(Node):
    """Emulates the onboard ESP32 state machine (see docs/ARCHITECTURE.md)."""

    def __init__(self):
        super().__init__('sim_esp32_emulator_node')
        cal = load_calibration()
        self.linear_speed = self.declare_parameter(
            'linear_speed', float(cal['linear_speed'])).value
        self.angular_speed = self.declare_parameter(
            'angular_speed', float(cal['angular_speed'])).value
        self.k_yaw = self.declare_parameter('k_yaw', 1.5).value
        self.pass_distance = self.declare_parameter('pass_distance', 0.35).value
        self.goal_hold_sec = self.declare_parameter('goal_hold_sec', 3.0).value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(UInt8, '/rover_status', 10)
        self.create_subscription(Float32, '/gs_command', self.on_command, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(UInt8, '/obstacle_state', self.on_obstacle, 10)
        self.create_timer(0.05, self.on_timer)  # 20 Hz

        self.pose: Optional[tuple] = None  # (x, y, yaw)
        self.obstacle = CLEAR
        self.status = STANDBY
        self.target = 0.0
        self.start_pose = (0.0, 0.0, 0.0)
        self.dist_along = 0.0
        self.pass_pending = False       # an avoid-turn happened; a forward pass is owed
        self.pass_start: Optional[tuple] = None
        self.goal_time = None
        self.get_logger().info('ESP32 emulator ready. Publish Float32 to /gs_command.')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_obstacle(self, msg: UInt8):
        self.obstacle = msg.data

    def on_command(self, msg: Float32):
        if msg.data <= 0.0:
            self.get_logger().warn(f'Ignoring non-positive target {msg.data}')
            return
        if self.pose is None:
            self.get_logger().warn('No /odom yet; ignoring command')
            return
        self.start_pose = self.pose
        self.target = float(msg.data)
        self.dist_along = 0.0
        self.pass_pending = False
        self.pass_start = None
        self.goal_time = None
        self.status = EXECUTING
        self.get_logger().info(
            f'Advance {self.target:.2f} m from x={self.pose[0]:.2f} y={self.pose[1]:.2f} '
            f'yaw={math.degrees(self.pose[2]):.1f} deg')

    def _publish(self, lin: float, ang: float):
        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.cmd_pub.publish(cmd)
        self.status_pub.publish(UInt8(data=self.status))

    def on_timer(self):
        now = self.get_clock().now()
        if self.status == GOAL_REACHED:
            self._publish(0.0, 0.0)
            if self.goal_time is not None and (now - self.goal_time).nanoseconds * 1e-9 \
                    >= self.goal_hold_sec:
                self.status = STANDBY
            return
        if self.status == STANDBY or self.pose is None:
            self._publish(0.0, 0.0)
            return

        x, y, yaw = self.pose
        x0, y0, yaw0 = self.start_pose
        self.dist_along = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)

        if self.dist_along >= self.target:
            self.status = GOAL_REACHED
            self.goal_time = now
            self.get_logger().info(f'GOAL_REACHED: progress {self.dist_along:.2f} m')
            self._publish(0.0, 0.0)
            return

        obs = self.obstacle
        lin, ang = 0.0, 0.0
        if obs == STOP:
            self.status = EXECUTING
        elif obs in (AVOID_LEFT, AVOID_RIGHT):
            self.status = AVOIDING
            self.pass_pending = True
            self.pass_start = None
            ang = self.angular_speed if obs == AVOID_LEFT else -self.angular_speed
        else:
            self.status = EXECUTING
            if self.pass_pending:
                if self.pass_start is None:
                    self.pass_start = (x, y)
                travelled = math.hypot(x - self.pass_start[0], y - self.pass_start[1])
                if travelled >= self.pass_distance:
                    self.pass_pending = False
                    self.pass_start = None
                lin = self.linear_speed  # straight along the deflected heading
            elif obs == HOLD_HEADING:
                lin = self.linear_speed
            else:  # CLEAR: realign with the original tunnel heading
                lin = self.linear_speed
                ang = max(-self.angular_speed,
                          min(self.angular_speed, self.k_yaw * wrap_angle(yaw0 - yaw)))
        self._publish(lin, ang)


def main(args=None):
    rclpy.init(args=args)
    node = SimEsp32Emulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
