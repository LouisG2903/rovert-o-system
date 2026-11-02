"""Unit tests: Packet 1/2/3 framing, tick clamp, map compress/decode, mock-serial bridge."""
import os
import sys
import time

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import ByteMultiArray, UInt8

from roverto_autonomy.esp32_bridge_node import Esp32BridgeNode
from roverto_autonomy.map_compressor_node import compress_grid, frame_chunks
from roverto_autonomy.protocol import (
    bytes_to_msg_data, pack_packet1, pack_packet2, pack_packet3, Packet1Parser,
    unpack_packet3, xor_checksum)

GS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..',
                                  'ground_station_gui'))
sys.path.insert(0, GS)
from gs_map_decoder import MapDecoder  # noqa: E402


def test_packet1_roundtrip_and_resync():
    p = Packet1Parser()
    good = pack_packet1(-1234, 5678, 2)
    assert len(good) == 12
    bad = bytearray(pack_packet1(1, 2, 1))
    bad[-1] ^= 0xFF
    out = p.feed(b'\x00\x13' + bytes(bad) + b'\xAA' + good[:5])
    out += p.feed(good[5:])
    assert out == [(-1234, 5678, 2)]
    assert p.bad_checksums == 1


def test_packet2_and_3_framing():
    p2 = pack_packet2(3)
    assert len(p2) == 4 and p2[:2] == b'\xAA\x66' and p2[3] == xor_checksum(p2[:3])
    f = pack_packet3(7, 1, 4, b'hello')
    assert f[:6] == bytes([0xAA, 0x77, 7, 1, 4, 5]) and len(f) == 12
    assert unpack_packet3(f) == (7, 1, 4, b'hello')
    assert unpack_packet3(f[:-1] + bytes([f[-1] ^ 1])) is None
    with pytest.raises(ValueError):
        pack_packet3(0, 0, 1, bytes(199))


def test_compress_decode_roundtrip():
    h, w = 40, 60
    data = -np.ones((h, w), dtype=np.int8)
    data[5:30, 10:50] = 0
    data[5, 10:50] = 100
    data[12, 20] = 49
    data[13, 21] = 50
    raw, comp = compress_grid(w, h, 0.05, -1.0, -2.0, data.ravel())
    frames = frame_chunks(comp, 9)
    assert all(len(fr) <= 205 for fr in frames)
    dec, res = MapDecoder(), None
    for fr in frames:
        res = dec.parse_packet(fr)
    grid, meta = res
    assert (meta['width'], meta['height'], meta['resolution_mm']) == (40, 25, 50)
    assert (meta['origin_x_cm'], meta['origin_y_cm']) == (-50, -175)
    exp = np.where(data[5:30, 10:50] == -1, 0, np.where(data[5:30, 10:50] >= 50, 2, 1))
    assert (grid == exp).all()


def test_decoder_incomplete_map_reset():
    import logging
    records = []

    class H(logging.Handler):
        def emit(self, record):
            records.append(record.getMessage())
    handler = H()
    logging.getLogger('gs_map_decoder').addHandler(handler)
    data = np.zeros((300, 300), dtype=np.int8)
    data[::7, ::5] = 100
    rng = np.random.default_rng(0)
    data[rng.random(data.shape) > 0.5] = -1
    data[0, 0] = 0
    _, comp = compress_grid(300, 300, 0.05, 0, 0, data.ravel())
    f_old = frame_chunks(comp, 1)
    f_new = frame_chunks(comp, 2)
    assert len(f_old) > 1
    dec = MapDecoder()
    assert dec.parse_packet(f_old[0]) is None
    for fr in f_new:
        res = dec.parse_packet(fr)
    logging.getLogger('gs_map_decoder').removeHandler(handler)
    assert res is not None and any('Incomplete map seq=1' in r for r in records)


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin(node, secs):
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def test_bridge_mock_loopback(ros):
    node = Esp32BridgeNode(parameter_overrides=[
        rclpy.parameter.Parameter('mock_serial', value=True)])
    odoms = []
    helper = rclpy.create_node('bridge_test_helper')
    helper.create_subscription(Odometry, '/odom', lambda m: odoms.append(m), 10)
    obs_pub = helper.create_publisher(UInt8, '/obstacle_state', 10)
    chunk_pub = helper.create_publisher(ByteMultiArray, '/compressed_map_chunk', 100)
    ser = node.ser
    tpm = node.tpm

    ser.inject(pack_packet1(0, 0, 0))
    ser.inject(pack_packet1(int(0.5 * tpm), int(0.5 * tpm), 1))   # 0.5 m straight
    _spin(node, 0.1)
    ser.inject(pack_packet1(int(0.5 * tpm) + 100000, int(0.5 * tpm), 1))  # jump -> clamp
    ser.inject(pack_packet1(int(1.0 * tpm), int(1.0 * tpm), 1))  # jump too (relative to jump)
    for _ in range(30):
        _spin(node, 0.01)
        rclpy.spin_once(helper, timeout_sec=0.0)
    assert node.odom_calc.x == pytest.approx(0.5, abs=1e-3)       # jumps ignored
    assert node.odom_calc.y == pytest.approx(0.0, abs=1e-6)

    obs_pub.publish(UInt8(data=3))
    for i in range(4):
        chunk_pub.publish(ByteMultiArray(data=bytes_to_msg_data(
            pack_packet3(0, i, 4, bytes([i]) * 50))))
    for _ in range(60):
        rclpy.spin_once(helper, timeout_sec=0.0)
        _spin(node, 0.01)
    p2 = [w for _, w in ser.writes if len(w) == 4]
    assert p2 and any(w == pack_packet2(3) for w in p2)
    p3 = [(t, w) for t, w in ser.writes if len(w) == 57]
    assert [unpack_packet3(w)[1] for _, w in p3] == [0, 1, 2, 3]
    gaps = [b[0] - a[0] for a, b in zip(p3, p3[1:])]
    assert all(g >= 0.020 for g in gaps), gaps                    # 25 ms pacing (timer jitter)
    helper.destroy_node()
    node.destroy_node()

