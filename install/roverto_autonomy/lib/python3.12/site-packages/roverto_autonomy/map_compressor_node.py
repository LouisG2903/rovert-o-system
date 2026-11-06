"""Compress /map into Packet 3 chunks (crop + 2-bit quantisation + zlib) for ESP-NOW telemetry."""
import struct
import zlib

import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ByteMultiArray, UInt8

from roverto_autonomy.protocol import bytes_to_msg_data, P3_MAX_PAYLOAD, pack_packet3

GOAL_REACHED = 3
META_FMT = '<HHHhh'  # width, height, resolution_mm, origin_x_cm, origin_y_cm (10 bytes)


def compress_grid(width, height, resolution, origin_x, origin_y, data):
    """Crop to known cells, quantise to 2 bits (4 cells/byte), prepend header, zlib level 9.

    Returns (raw_bytes, compressed_bytes) or None when the map holds no known cells.
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    known = grid != -1
    if not known.any():
        return None
    rows = np.where(known.any(axis=1))[0]
    cols = np.where(known.any(axis=0))[0]
    r0, r1, c0, c1 = rows[0], rows[-1], cols[0], cols[-1]
    crop = grid[r0:r1 + 1, c0:c1 + 1]

    cells = np.zeros(crop.shape, dtype=np.uint8)          # 0 = unknown
    cells[(crop >= 0) & (crop <= 49)] = 1                 # free
    cells[(crop >= 50) & (crop <= 100)] = 2               # occupied
    flat = cells.ravel()
    pad = (-flat.size) % 4
    if pad:
        flat = np.concatenate([flat, np.zeros(pad, dtype=np.uint8)])
    q = flat.reshape(-1, 4)
    packed = ((q[:, 0] << 6) | (q[:, 1] << 4) | (q[:, 2] << 2) | q[:, 3]).astype(np.uint8)

    ox_cm = int(round((origin_x + c0 * resolution) * 100.0))
    oy_cm = int(round((origin_y + r0 * resolution) * 100.0))
    ox_cm = max(-32768, min(32767, ox_cm))
    oy_cm = max(-32768, min(32767, oy_cm))
    header = struct.pack(META_FMT, crop.shape[1], crop.shape[0],
                         int(round(resolution * 1000.0)), ox_cm, oy_cm)
    raw = header + packed.tobytes()
    return raw, zlib.compress(raw, 9)


def frame_chunks(compressed: bytes, seq: int):
    """Slice into Packet 3 frames (<=198 B payload each). Returns list of frames or None."""
    n = max(1, -(-len(compressed) // P3_MAX_PAYLOAD))
    if n > 255:
        return None
    return [pack_packet3(seq, i, n, compressed[i * P3_MAX_PAYLOAD:(i + 1) * P3_MAX_PAYLOAD])
            for i in range(n)]


class MapCompressorNode(Node):
    """Publishes Packet 3 frames on /compressed_map_chunk."""

    def __init__(self):
        super().__init__('map_compressor_node')
        self.period = self.declare_parameter('periodic_sec', 10.0).value
        # slam_toolbox rebuilds /map every 2 s; wait a little longer so the goal map is complete.
        self.goal_delay = self.declare_parameter('goal_map_delay_sec', 2.5).value

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        chunk_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(ByteMultiArray, '/compressed_map_chunk', chunk_qos)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(UInt8, '/rover_status', self.on_status, 10)
        self.create_timer(self.period, self.on_periodic)

        self.latest = None
        self.dirty = False        # map changed since last transmission
        self.seq = 0
        self.prev_status = None
        self._goal_timer = None

    def on_map(self, msg: OccupancyGrid):
        if self.latest is None or list(msg.data) != list(self.latest.data) \
                or msg.info.width != self.latest.info.width \
                or msg.info.height != self.latest.info.height:
            self.dirty = True
        self.latest = msg

    def on_status(self, msg: UInt8):
        if msg.data == GOAL_REACHED and self.prev_status != GOAL_REACHED:
            self.get_logger().info(
                f'GOAL_REACHED: sending map in {self.goal_delay:.1f} s (let SLAM finish updating)')
            if self._goal_timer is not None:
                self._goal_timer.cancel()
            self._goal_timer = self.create_timer(self.goal_delay, self._on_goal_timer)
        self.prev_status = msg.data

    def _on_goal_timer(self):
        self._goal_timer.cancel()
        self._goal_timer = None
        self.send_map('goal')

    def on_periodic(self):
        if self.dirty:
            self.send_map('periodic')

    def send_map(self, reason: str):
        if self.latest is None:
            self.get_logger().warn('No /map received yet; nothing to send')
            return
        m = self.latest
        result = compress_grid(m.info.width, m.info.height, m.info.resolution,
                               m.info.origin.position.x, m.info.origin.position.y, m.data)
        if result is None:
            self.get_logger().warn('Map has no known cells; skipping')
            return
        raw, comp = result
        frames = frame_chunks(comp, self.seq)
        if frames is None:
            self.get_logger().error(f'Compressed map {len(comp)} B needs >255 chunks; skipped')
            return
        for f in frames:
            out = ByteMultiArray()
            out.data = bytes_to_msg_data(f)
            self.pub.publish(out)
        self.get_logger().info(
            f'[{reason}] map_seq={self.seq}: raw {len(raw)} B -> zlib {len(comp)} B '
            f'(ratio {len(raw) / len(comp):.1f}:1), {len(frames)} chunks')
        self.seq = (self.seq + 1) & 0xFF
        self.dirty = False


def main(args=None):
    rclpy.init(args=args)
    node = MapCompressorNode()
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
