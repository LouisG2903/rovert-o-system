"""Binary UART protocol helpers (see docs/ARCHITECTURE.md). Little-endian, XOR checksums.

Packet 1 (ESP32 -> Pi, 12 B):  <BBiiBB  0xAA 0x55 left_ticks right_ticks status xor
Packet 2 (Pi -> ESP32, 4 B):   <BBBB    0xAA 0x66 obstacle_state xor
Packet 3 (Pi -> ESP32, <=205): <BBBBBB  0xAA 0x77 seq chunk total len + payload + xor
"""
import struct
from typing import List, Optional, Tuple

SYNC = 0xAA
P1_ID, P2_ID, P3_ID = 0x55, 0x66, 0x77
P1_FMT = '<BBiiBB'
P1_LEN = struct.calcsize(P1_FMT)          # 12
P2_FMT = '<BBBB'
P2_LEN = struct.calcsize(P2_FMT)          # 4
P3_HDR_FMT = '<BBBBBB'
P3_HDR_LEN = struct.calcsize(P3_HDR_FMT)  # 6
P3_MAX_PAYLOAD = 198


def xor_checksum(data: bytes) -> int:
    """XOR of all bytes."""
    c = 0
    for b in data:
        c ^= b
    return c


def pack_packet1(left_ticks: int, right_ticks: int, status: int) -> bytes:
    """Build an ESP32 -> Pi packet (used for tests / ESP32 emulation)."""
    body = struct.pack('<BBiiB', SYNC, P1_ID, left_ticks, right_ticks, status)
    return body + bytes([xor_checksum(body)])


def pack_packet2(obstacle_state: int) -> bytes:
    """Build a Pi -> ESP32 obstacle-state packet."""
    body = struct.pack('<BBB', SYNC, P2_ID, obstacle_state)
    return body + bytes([xor_checksum(body)])


def pack_packet3(seq: int, chunk_id: int, total: int, payload: bytes) -> bytes:
    """Build a Pi -> ESP32 map chunk frame."""
    if len(payload) > P3_MAX_PAYLOAD:
        raise ValueError(f'payload {len(payload)} > {P3_MAX_PAYLOAD}')
    body = struct.pack(P3_HDR_FMT, SYNC, P3_ID, seq & 0xFF, chunk_id, total, len(payload)) \
        + payload
    return body + bytes([xor_checksum(body)])


def unpack_packet3(frame: bytes) -> Optional[Tuple[int, int, int, bytes]]:
    """Validate a Packet 3 frame. Returns (seq, chunk_id, total, payload) or None."""
    if len(frame) < P3_HDR_LEN + 1:
        return None
    sync1, sync2, seq, chunk_id, total, plen = struct.unpack(P3_HDR_FMT, frame[:P3_HDR_LEN])
    if sync1 != SYNC or sync2 != P3_ID or plen > P3_MAX_PAYLOAD:
        return None
    if len(frame) != P3_HDR_LEN + plen + 1:
        return None
    if xor_checksum(frame[:-1]) != frame[-1]:
        return None
    return seq, chunk_id, total, bytes(frame[P3_HDR_LEN:P3_HDR_LEN + plen])


class Packet1Parser:
    """Non-blocking byte-wise state machine: sync on 0xAA 0x55, verify XOR checksum."""

    def __init__(self):
        self._buf = bytearray()
        self.bad_checksums = 0

    def feed(self, data: bytes) -> List[Tuple[int, int, int]]:
        """Consume bytes; return list of (left_ticks, right_ticks, status) for valid packets."""
        out = []
        for byte in data:
            n = len(self._buf)
            if n == 0:
                if byte == SYNC:
                    self._buf.append(byte)
                continue
            if n == 1:
                if byte == P1_ID:
                    self._buf.append(byte)
                elif byte == SYNC:
                    pass  # stay on a fresh 0xAA
                else:
                    self._buf.clear()
                continue
            self._buf.append(byte)
            if len(self._buf) == P1_LEN:
                frame = bytes(self._buf)
                self._buf.clear()
                if xor_checksum(frame[:-1]) == frame[-1]:
                    _, _, left, right, status, _ = struct.unpack(P1_FMT, frame)
                    out.append((left, right, status))
                else:
                    self.bad_checksums += 1
                    # A real sync pair may hide inside the rejected bytes; resync from them.
                    out.extend(self.feed(frame[1:]))
        return out


def msg_data_to_bytes(data) -> bytes:
    """Normalise ByteMultiArray.data (bytes, list of ints or list of 1-byte bytes) to bytes."""
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    return b''.join(bytes([x]) if isinstance(x, int) else bytes(x) for x in data)


def bytes_to_msg_data(frame: bytes) -> list:
    """Convert bytes to the ByteMultiArray.data representation (list of 1-byte bytes)."""
    return [bytes([b]) for b in frame]
