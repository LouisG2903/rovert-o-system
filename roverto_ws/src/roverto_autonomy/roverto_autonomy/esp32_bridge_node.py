"""ESP32 UART bridge: Packet 1 -> /odom + TF + /rover_status; Packet 2/3 <- Pi."""
from collections import deque
import math
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ByteMultiArray, UInt8
from tf2_ros import TransformBroadcaster

from roverto_autonomy.calibration import load_calibration
from roverto_autonomy.protocol import (
    msg_data_to_bytes, pack_packet2, Packet1Parser, unpack_packet3)

try:
    import serial  # pyserial (apt: python3-serial)
except ImportError:  # handled at runtime: node stays alive and reports the problem
    serial = None


class MockSerial:
    """In-memory loopback port. inject() feeds bytes the node will read (ESP32 -> Pi);
    everything the node writes (Pi -> ESP32) is recorded with a timestamp in .writes."""

    def __init__(self):
        self._rx = bytearray()
        self.writes = []  # list of (monotonic_time, bytes)
        self.is_open = True

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def inject(self, data: bytes):
        self._rx.extend(data)

    def read(self, n: int = 1) -> bytes:
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def write(self, data: bytes) -> int:
        self.writes.append((time.monotonic(), bytes(data)))
        return len(data)

    def close(self):
        self.is_open = False

    @property
    def written(self) -> bytes:
        return b''.join(w for _, w in self.writes)


class DiffDriveOdometry:
    """Midpoint (2nd-order Runge-Kutta) differential-drive integration."""

    def __init__(self, ticks_per_meter: float, wheel_separation: float):
        self.tpm = ticks_per_meter
        self.sep = wheel_separation
        self.x = self.y = self.yaw = 0.0

    def update(self, dl_ticks: int, dr_ticks: int):
        """Integrate tick deltas. Returns (distance, dtheta) for this step."""
        dl = dl_ticks / self.tpm
        dr = dr_ticks / self.tpm
        d = 0.5 * (dl + dr)
        dth = (dr - dl) / self.sep
        mid = self.yaw + 0.5 * dth
        self.x += d * math.cos(mid)
        self.y += d * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dth), math.cos(self.yaw + dth))
        return d, dth


class Esp32BridgeNode(Node):
    """See docs/ARCHITECTURE.md for packet formats."""

    CHUNK_INTERVAL = 0.025  # s, minimum spacing between Packet 3 writes

    def __init__(self, **node_kwargs):
        super().__init__('esp32_bridge_node', **node_kwargs)
        cal = load_calibration()
        self.port = self.declare_parameter('serial_port', '/dev/ttyAMA0').value
        self.baud = self.declare_parameter('baud_rate', 115200).value
        self.tpm = self.declare_parameter('ticks_per_meter', float(cal['ticks_per_meter'])).value
        self.sep = self.declare_parameter('wheel_separation', float(cal['wheel_separation'])).value
        self.max_ticks = self.declare_parameter(
            'max_ticks_per_cycle', int(cal['max_ticks_per_cycle'])).value
        self.mock = self.declare_parameter('mock_serial', False).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value

        self.odom_calc = DiffDriveOdometry(self.tpm, self.sep)
        self.parser = Packet1Parser()
        self.prev_ticks = None
        self.last_stamp = None
        self.obstacle_state = 0
        self.map_queue = deque(maxlen=4000)
        self.ser = None
        self._open_serial()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(UInt8, '/rover_status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(UInt8, '/obstacle_state', self.on_obstacle, 10)
        self.create_subscription(
            ByteMultiArray, '/compressed_map_chunk', self.on_chunk,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.01, self.on_read)                  # 100 Hz non-blocking read
        self.create_timer(0.2, self.on_keepalive)              # 5 Hz Packet 2 keepalive
        self.create_timer(self.CHUNK_INTERVAL, self.on_chunk_timer)  # paced Packet 3
        self.create_timer(2.0, self._reconnect_check)
        self.get_logger().info(
            f'ESP32 bridge: {"MOCK" if self.mock else self.port}@{self.baud}, '
            f'ticks/m={self.tpm}, sep={self.sep} m, clamp={self.max_ticks} ticks')

    # ---- serial plumbing ---------------------------------------------------------------
    def _open_serial(self):
        if self.mock:
            self.ser = MockSerial()
            return
        if serial is None:
            self.get_logger().error('pyserial missing: install python3-serial. UART disabled.')
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.1)
            self.get_logger().info(f'Opened {self.port}')
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().error(f'Cannot open {self.port}: {e} (will retry)')

    def _reconnect_check(self):
        if self.ser is None and not self.mock and serial is not None:
            self._open_serial()

    def _write(self, data: bytes):
        if self.ser is None:
            return
        try:
            self.ser.write(data)
        except Exception as e:  # serial.SerialException, timeouts, unplugged device
            self.get_logger().error(f'UART write failed: {e}')
            try:
                self.ser.close()
            finally:
                self.ser = None

    # ---- ESP32 -> ROS ------------------------------------------------------------------
    def on_read(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n) if n else b''
        except Exception as e:
            self.get_logger().error(f'UART read failed: {e}')
            self.ser = None
            return
        for left, right, status in self.parser.feed(data):
            self.handle_packet1(left, right, status)

    def handle_packet1(self, left: int, right: int, status: int):
        now = self.get_clock().now()
        self.status_pub.publish(UInt8(data=status))
        if self.prev_ticks is None:
            self.prev_ticks = (left, right)
            self.last_stamp = now
            self._publish_odom(now, 0.0, 0.0)
            return
        dl, dr = left - self.prev_ticks[0], right - self.prev_ticks[1]
        self.prev_ticks = (left, right)
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now
        if abs(dl) > self.max_ticks or abs(dr) > self.max_ticks:
            self.get_logger().warn('ESP32 tick jump/reset detected — clamping delta to 0')
            self._publish_odom(now, 0.0, 0.0)
            return
        d, dth = self.odom_calc.update(dl, dr)
        v, w = (d / dt, dth / dt) if dt > 1e-6 else (0.0, 0.0)
        self._publish_odom(now, v, w)

    def _publish_odom(self, now, v: float, w: float):
        stamp = now.to_msg()
        x, y, yaw = self.odom_calc.x, self.odom_calc.y, self.odom_calc.yaw
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        cov = [0.0] * 36
        for i, val in ((0, 0.01), (7, 0.01), (14, 1e6), (21, 1e6), (28, 1e6), (35, 0.03)):
            cov[i] = val
        odom.pose.covariance = cov
        tcov = list(cov)
        tcov[0], tcov[7], tcov[35] = 0.01, 0.01, 0.03
        odom.twist.covariance = tcov
        self.odom_pub.publish(odom)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # ---- ROS -> ESP32 ------------------------------------------------------------------
    def on_obstacle(self, msg: UInt8):
        changed = msg.data != self.obstacle_state
        self.obstacle_state = msg.data
        if changed:
            self._write(pack_packet2(self.obstacle_state))

    def on_keepalive(self):
        self._write(pack_packet2(self.obstacle_state))

    def on_chunk(self, msg: ByteMultiArray):
        frame = msg_data_to_bytes(msg.data)
        if unpack_packet3(frame) is None:
            self.get_logger().warn(f'Dropping invalid Packet 3 frame ({len(frame)} B)')
            return
        if len(self.map_queue) == self.map_queue.maxlen:
            self.get_logger().warn('Map chunk queue full; dropping oldest frame')
        self.map_queue.append(frame)

    def on_chunk_timer(self):
        if self.map_queue:
            self._write(self.map_queue.popleft())


def main(args=None):
    rclpy.init(args=args)
    node = Esp32BridgeNode()
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
