"""Reactive LiDAR guard: classifies /scan into an obstacle_state (UInt8) for the rover."""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8

from roverto_autonomy.calibration import load_calibration

CLEAR, AVOID_LEFT, AVOID_RIGHT, STOP, HOLD_HEADING = 0, 1, 2, 3, 4

DEG = math.pi / 180.0
FRONT = (-30.0 * DEG, 30.0 * DEG)
LEFT_ASSESS = (15.0 * DEG, 80.0 * DEG)
RIGHT_ASSESS = (-80.0 * DEG, -15.0 * DEG)
LEFT_FLANK = (35.0 * DEG, 105.0 * DEG)
RIGHT_FLANK = (-105.0 * DEG, -35.0 * DEG)


def sector_ranges(angles, ranges, sector):
    """Return the valid ranges whose (wrapped) angle lies in [sector[0], sector[1]]."""
    mask = (angles >= sector[0]) & (angles <= sector[1])
    return ranges[mask]


class LidarGuardNode(Node):
    """Publishes /obstacle_state from /scan (0 CLEAR, 1 LEFT, 2 RIGHT, 3 STOP, 4 HOLD_HEADING)."""

    def __init__(self):
        super().__init__('lidar_guard_node')
        cal = load_calibration()
        self.front_stop = self.declare_parameter(
            'front_stop_distance', float(cal['front_stop_distance'])).value
        self.flank_safe = self.declare_parameter(
            'flank_safe_distance', float(cal['flank_safe_distance'])).value
        self.pub = self.create_publisher(UInt8, '/obstacle_state', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self._last_state = None
        # Latched AVOID_LEFT/AVOID_RIGHT: the turn direction is kept until the front cone has
        # been clear for avoid_release_sec (prevents left/right dithering at cone edges).
        self.release_sec = self.declare_parameter('avoid_release_sec', 0.5).value
        self._avoid_dir = None
        self._clear_since = None
        self.get_logger().info(
            f'front_stop={self.front_stop:.2f} m, flank_safe={self.flank_safe:.2f} m')

    @staticmethod
    def _min(vals):
        return float(vals.min()) if vals.size else math.inf

    @staticmethod
    def _clearance(vals, range_max):
        # Mean of valid returns; no return at all means fully open.
        return float(vals.mean()) if vals.size else float(range_max)

    def classify(self, msg: LaserScan) -> int:
        n = len(msg.ranges)
        if n == 0:
            return CLEAR
        raw = msg.angle_min + np.arange(n) * msg.angle_increment
        angles = np.arctan2(np.sin(raw), np.cos(raw))  # [-pi, pi], 0 = +X, + = left
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        angles, ranges = angles[valid], ranges[valid]

        front_min = self._min(sector_ranges(angles, ranges, FRONT))
        left_a = sector_ranges(angles, ranges, LEFT_ASSESS)
        right_a = sector_ranges(angles, ranges, RIGHT_ASSESS)

        if front_min < self.front_stop:
            if self._min(left_a) < self.flank_safe and self._min(right_a) < self.flank_safe:
                self._avoid_dir = None
                self._clear_since = None
                return STOP
            self._clear_since = None
            if self._avoid_dir is None:
                left_clear = self._clearance(left_a, msg.range_max)
                right_clear = self._clearance(right_a, msg.range_max)
                self._avoid_dir = AVOID_LEFT if left_clear >= right_clear else AVOID_RIGHT
            return self._avoid_dir

        if self._avoid_dir is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._clear_since is None:
                self._clear_since = now
            if now - self._clear_since < self.release_sec:
                return self._avoid_dir  # keep turning briefly to clear the obstacle edge
            self._avoid_dir = None
            self._clear_since = None
        flank_min = min(self._min(sector_ranges(angles, ranges, LEFT_FLANK)),
                        self._min(sector_ranges(angles, ranges, RIGHT_FLANK)))
        return HOLD_HEADING if flank_min < self.flank_safe else CLEAR

    def on_scan(self, msg: LaserScan):
        # Allow live tuning through ROS parameters.
        self.front_stop = self.get_parameter('front_stop_distance').value
        self.flank_safe = self.get_parameter('flank_safe_distance').value
        state = self.classify(msg)
        self.pub.publish(UInt8(data=state))
        if state != self._last_state:
            self.get_logger().info(f'obstacle_state -> {state}')
            self._last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = LidarGuardNode()
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
