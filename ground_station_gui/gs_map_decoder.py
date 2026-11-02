#!/usr/bin/env python3
"""Ground-station decoder for Rovert-O compressed map telemetry (Packet 3).

Library use:
    dec = MapDecoder()
    result = dec.parse_packet(frame_bytes)      # None until a map is complete
    if result: grid, meta = result; save_map_png(grid, meta, 'map.png')

CLI:
    gs_map_decoder.py --bin frames.bin -o map.png   # concatenated Packet 3 frames
    gs_map_decoder.py --ros -o map.png              # subscribe to /compressed_map_chunk (ROS 2)

Packet 3: <BBBBBB 0xAA 0x77 map_seq_id chunk_id total_chunks payload_len (<=198)
          + payload + XOR checksum (XOR of every preceding byte).
Reassembled payload = zlib( <HHHhh width height resolution_mm origin_x_cm origin_y_cm
                            + 2-bit cells, row-major, 4 per byte, MSB first ).
Cell codes: 0 unknown, 1 free, 2 occupied.
"""
import argparse
import logging
import struct
import sys
import time
import zlib

import numpy as np

log = logging.getLogger('gs_map_decoder')

SYNC1, SYNC2 = 0xAA, 0x77
HDR_FMT = '<BBBBBB'
HDR_LEN = struct.calcsize(HDR_FMT)
MAX_PAYLOAD = 198
META_FMT = '<HHHhh'
META_LEN = struct.calcsize(META_FMT)  # 10
UNKNOWN, FREE, OCCUPIED = 0, 1, 2


def xor_checksum(data: bytes) -> int:
    """XOR of all bytes."""
    c = 0
    for b in data:
        c ^= b
    return c


def iter_frames(stream: bytes):
    """Split a concatenation of Packet 3 frames using each header's payload_len."""
    i = 0
    while i + HDR_LEN <= len(stream):
        if stream[i] != SYNC1 or stream[i + 1] != SYNC2:
            i += 1
            continue
        end = i + HDR_LEN + stream[i + 5] + 1
        if end > len(stream):
            break
        yield stream[i:end]
        i = end


class MapDecoder:
    """Reassembles chunked, compressed maps. Feed frames with parse_packet()."""

    def __init__(self, timeout_s: float = 10.0):
        self.timeout_s = timeout_s
        self.reset()

    def reset(self):
        self.seq = None
        self.total = 0
        self.chunks = {}
        self.last_rx = 0.0

    def check_timeout(self) -> bool:
        """Drop a partial map that stalled for longer than timeout_s. True if dropped."""
        if self.seq is not None and time.monotonic() - self.last_rx > self.timeout_s:
            log.warning('Incomplete map seq=%d (%d/%d chunks) timed out; discarded',
                        self.seq, len(self.chunks), self.total)
            self.reset()
            return True
        return False

    def parse_packet(self, frame: bytes):
        """Validate one frame. Returns (grid, metadata) when a map completes, else None."""
        frame = bytes(frame)
        if len(frame) < HDR_LEN + 1:
            log.warning('Frame too short (%d B)', len(frame))
            return None
        s1, s2, seq, chunk_id, total, plen = struct.unpack(HDR_FMT, frame[:HDR_LEN])
        if s1 != SYNC1 or s2 != SYNC2:
            log.warning('Bad header %02X %02X', s1, s2)
            return None
        if plen > MAX_PAYLOAD or len(frame) != HDR_LEN + plen + 1:
            log.warning('Bad length: payload_len=%d, frame=%d B', plen, len(frame))
            return None
        if xor_checksum(frame[:-1]) != frame[-1]:
            log.warning('Checksum mismatch (seq=%d chunk=%d)', seq, chunk_id)
            return None
        if total == 0 or chunk_id >= total:
            log.warning('Invalid chunk index %d/%d', chunk_id, total)
            return None

        self.check_timeout()
        if self.seq is not None and seq != self.seq:
            log.warning('Incomplete map seq=%d (%d/%d chunks) abandoned: new map seq=%d began',
                        self.seq, len(self.chunks), self.total, seq)
            self.reset()
        if self.seq is None:
            self.seq, self.total = seq, total
        elif total != self.total:
            log.warning('total_chunks changed mid-map (seq=%d); restarting', seq)
            self.reset()
            self.seq, self.total = seq, total
        self.last_rx = time.monotonic()
        self.chunks[chunk_id] = frame[HDR_LEN:HDR_LEN + plen]

        if len(self.chunks) < self.total:
            return None
        blob = b''.join(self.chunks[i] for i in range(self.total))
        done_seq = self.seq
        self.reset()
        try:
            return decode_map(blob)
        except (zlib.error, ValueError) as e:
            log.error('Map seq=%d failed to decode: %s', done_seq, e)
            return None


def decode_map(compressed: bytes):
    """zlib-decompress, unpack the 10-byte header and 2-bit cells. Returns (grid, metadata)."""
    raw = zlib.decompress(compressed)
    if len(raw) < META_LEN:
        raise ValueError('decompressed buffer shorter than metadata header')
    width, height, res_mm, ox_cm, oy_cm = struct.unpack(META_FMT, raw[:META_LEN])
    n = width * height
    packed = np.frombuffer(raw, dtype=np.uint8, offset=META_LEN)
    if packed.size * 4 < n:
        raise ValueError(f'cell data too short for {width}x{height}')
    cells = np.empty(packed.size * 4, dtype=np.uint8)
    cells[0::4] = (packed >> 6) & 3
    cells[1::4] = (packed >> 4) & 3
    cells[2::4] = (packed >> 2) & 3
    cells[3::4] = packed & 3
    grid = cells[:n].reshape(height, width)  # row 0 = y_min (OccupancyGrid convention)
    meta = {'width': width, 'height': height, 'resolution_mm': res_mm,
            'origin_x_cm': ox_cm, 'origin_y_cm': oy_cm}
    return grid, meta


def save_map_png(grid, metadata, output_path):
    """Render Unknown=gray, Free=white, Occupied=black with metric axes (matplotlib).

    Falls back to PIL (no axes), then to a plain PGM file if neither is installed.
    Returns the path actually written.
    """
    grid = np.asarray(grid)
    res = metadata['resolution_mm'] / 1000.0
    x0, y0 = metadata['origin_x_cm'] / 100.0, metadata['origin_y_cm'] / 100.0
    h, w = grid.shape
    shade = np.array([128, 255, 0], dtype=np.uint8)  # unknown, free, occupied
    img = shade[np.clip(grid, 0, 2)]
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(max(4.0, w * res * 0.8), max(3.0, h * res * 0.8 + 1)))
        ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='lower',
                  extent=(x0, x0 + w * res, y0, y0 + h * res), interpolation='nearest')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title(f'Reconstructed map {w}x{h} @ {metadata["resolution_mm"]} mm/cell')
        ax.grid(True, color='tab:blue', alpha=0.25, linewidth=0.5)
        fig.tight_layout()
        fig.savefig(output_path, dpi=150)
        plt.close(fig)
        return output_path
    except ImportError:
        pass
    flipped = np.flipud(img)  # image row 0 = top = y_max
    try:
        from PIL import Image
        Image.fromarray(flipped, 'L').save(output_path)
        return output_path
    except ImportError:
        pgm = str(output_path).rsplit('.', 1)[0] + '.pgm'
        with open(pgm, 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (w, h) + flipped.tobytes())
        return pgm


def _run_bin(path, out):
    dec = MapDecoder()
    with open(path, 'rb') as f:
        stream = f.read()
    last = None
    for frame in iter_frames(stream):
        res = dec.parse_packet(frame)
        if res:
            last = res
            log.info('Map complete: %s', res[1])
    if last is None:
        log.error('No complete map in %s', path)
        return 1
    print('saved', save_map_png(last[0], last[1], out))
    return 0


def _run_ros(out):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import ByteMultiArray

    class GsNode(Node):
        def __init__(self):
            super().__init__('gs_map_decoder')
            self.dec = MapDecoder()
            self.count = 0
            self.create_subscription(
                ByteMultiArray, '/compressed_map_chunk', self.cb,
                QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))

        def cb(self, msg):
            data = msg.data
            frame = bytes(data) if isinstance(data, (bytes, bytearray)) else b''.join(
                bytes([x]) if isinstance(x, int) else bytes(x) for x in data)
            res = self.dec.parse_packet(frame)
            if res:
                self.count += 1
                print('map', self.count, res[1], '->', save_map_png(res[0], res[1], out))

    rclpy.init()
    node = GsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bin', help='file with concatenated Packet 3 frames')
    ap.add_argument('--ros', action='store_true', help='subscribe to /compressed_map_chunk')
    ap.add_argument('-o', '--output', default='/tmp/reconstructed_map.png')
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
    if args.bin:
        return _run_bin(args.bin, args.output)
    if args.ros:
        return _run_ros(args.output)
    ap.print_help()
    return 2


if __name__ == '__main__':
    sys.exit(main())
