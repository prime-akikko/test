"""UART frame layout for LDRobot LD14 / LD14P (LDS14 Mini) modules.

Primary sources:
- LD14 Development Manual V1.2 (RobotShop / LDRobot)
- LD14P Development Manual V0.2 (Waveshare / LDRobot)
- LD14P Datasheet (UART @ 230400)

Scan packets are identical across LD14 and LD14P: 47 bytes, header 0x54,
VerLen 0x2C (packet type 1, 12 points), little-endian fields, CRC-8 poly 0x4D.

Assumptions documented where manuals conflict or omit detail (see module
comments and the Project doc).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable, List, Optional, Sequence, Tuple, Union

from .crc import crc8

HEADER = 0x54
SCAN_VER_LEN = 0x2C  # upper 3 bits = type 1; lower 5 bits = 12 points
HEALTH_INFO_MARK = 0x1F
POINTS_PER_PACKET = 12
SCAN_PACKET_SIZE = 47  # 2 + 2 + 2 + 12*3 + 2 + 2 + 1
HEALTH_PACKET_SIZE = 6  # 0x54 0x1F err_lo err_hi mode crc
COMMAND_SIZE = 8  # 0x54 cmd 0x04 payload[4] crc

# Angle fields are in units of 0.01 degrees (manual).
ANGLE_SCALE = 100.0
TIMESTAMP_WRAP_MS = 30_000


class MotorCommand(IntEnum):
    """LD14P serial motor-control opcodes (Development Manual V0.2 §2.2)."""

    START = 0xA0
    STOP = 0xA1
    SET_SPEED = 0xA2
    QUERY_SPEED = 0xA3


class HealthMode(IntEnum):
    """LD14P health packet error mode (§2.3.1)."""

    NORMAL = 0x00
    WARN = 0x01
    ERROR = 0x02


# Bit meanings from LD14P Development Manual V0.2 §2.3.1.
HEALTH_FLAGS = {
    0: "laser_tube",
    1: "laser_tube_alt",  # manual lists Bit0 and Bit1 both as laser tube
    2: "receiver_board_voltage",
    3: "sensor",
    4: "encoder",
    5: "motor_or_locked_rotor",
    6: "blockage",
}


@dataclass(frozen=True)
class LidarPoint:
    """One interpolated measurement from a scan packet."""

    angle_deg: float
    distance_mm: int
    intensity: int

    @property
    def valid(self) -> bool:
        """True when distance is non-zero (common validity heuristic)."""
        return self.distance_mm > 0


@dataclass(frozen=True)
class ScanPacket:
    """Parsed 47-byte measurement packet."""

    speed_deg_s: int
    start_angle_deg: float
    end_angle_deg: float
    timestamp_ms: int
    points: Tuple[LidarPoint, ...]
    raw: bytes

    @property
    def speed_hz(self) -> float:
        return self.speed_deg_s / 360.0


@dataclass(frozen=True)
class HealthInfo:
    """Parsed LD14P health / fault packet (§2.3)."""

    error_code: int
    mode: HealthMode
    flags: Tuple[str, ...]
    raw: bytes

    @property
    def ok(self) -> bool:
        return self.mode == HealthMode.NORMAL and self.error_code == 0


class ProtocolError(ValueError):
    """Raised when a frame fails structural or CRC checks."""


def build_command(cmd: int, payload: bytes = b"\x00\x00\x00\x00") -> bytes:
    """Build an 8-byte LD14P control frame: 0x54 | cmd | 0x04 | payload[4] | crc8."""
    if not (0 <= cmd <= 0xFF):
        raise ValueError(f"command out of range: {cmd}")
    payload = (payload + b"\x00\x00\x00\x00")[:4]
    body = bytes([HEADER, cmd & 0xFF, 0x04]) + payload
    return body + bytes([crc8(body)])


def start_motor_command() -> bytes:
    """Reference: ``54 A0 04 00 00 00 00 5E``."""
    return build_command(MotorCommand.START)


def stop_motor_command() -> bytes:
    """Reference: ``54 A1 04 00 00 00 00 4A``."""
    return build_command(MotorCommand.STOP)


def set_speed_command(speed_deg_s: int) -> bytes:
    """Set target motor speed in degrees/second (LD14P §2.2.4).

    Typical scan rates are 2–8 Hz → 720–2880 deg/s. Example for 6 Hz:
    ``54 A2 04 70 08 00 00 A1`` (2160 deg/s).
    """
    if not (0 <= speed_deg_s <= 0xFFFF):
        raise ValueError(f"speed_deg_s out of uint16 range: {speed_deg_s}")
    payload = struct.pack("<H", speed_deg_s) + b"\x00\x00"
    return build_command(MotorCommand.SET_SPEED, payload)


def query_speed_command() -> bytes:
    """Reference: ``54 A3 04 00 00 00 00 62``."""
    return build_command(MotorCommand.QUERY_SPEED)


def _angles_from_raw(start_raw: int, end_raw: int, count: int) -> List[float]:
    """Linearly interpolate point angles (manual §3.2).

    Manual formula (0.01° units)::

        step = (end_angle - start_angle) / (len - 1)
        angle = start_angle + step * i

    We convert to degrees and handle 360° wrap when ``end < start``.
    """
    if count < 1:
        return []
    start = start_raw / ANGLE_SCALE
    end = end_raw / ANGLE_SCALE
    if count == 1:
        return [start % 360.0]
    # Wrap-aware delta so a packet crossing 0° still interpolates forward.
    delta = (end - start) % 360.0
    step = delta / (count - 1)
    return [(start + step * i) % 360.0 for i in range(count)]


def parse_scan_packet(data: bytes | bytearray | memoryview) -> ScanPacket:
    """Parse and CRC-check a 47-byte scan packet.

    Raises:
        ProtocolError: on length, header, VerLen, or CRC mismatch.
    """
    if len(data) != SCAN_PACKET_SIZE:
        raise ProtocolError(f"scan packet must be {SCAN_PACKET_SIZE} bytes, got {len(data)}")
    raw = bytes(data)
    if raw[0] != HEADER or raw[1] != SCAN_VER_LEN:
        raise ProtocolError(f"bad scan header: {raw[0]:02X} {raw[1]:02X}")
    if crc8(raw[:-1]) != raw[-1]:
        raise ProtocolError("scan packet CRC mismatch")

    speed, start_raw = struct.unpack_from("<HH", raw, 2)
    points: List[LidarPoint] = []
    offset = 6
    distances: List[int] = []
    intensities: List[int] = []
    for _ in range(POINTS_PER_PACKET):
        distance, intensity = struct.unpack_from("<HB", raw, offset)
        distances.append(distance)
        intensities.append(intensity)
        offset += 3
    end_raw, timestamp = struct.unpack_from("<HH", raw, offset)
    angles = _angles_from_raw(start_raw, end_raw, POINTS_PER_PACKET)
    for angle, distance, intensity in zip(angles, distances, intensities):
        points.append(LidarPoint(angle_deg=angle, distance_mm=distance, intensity=intensity))

    return ScanPacket(
        speed_deg_s=speed,
        start_angle_deg=(start_raw % 36000) / ANGLE_SCALE,
        end_angle_deg=(end_raw % 36000) / ANGLE_SCALE,
        timestamp_ms=timestamp % TIMESTAMP_WRAP_MS,
        points=tuple(points),
        raw=raw,
    )


def parse_health_packet(data: bytes | bytearray | memoryview) -> HealthInfo:
    """Parse LD14P 6-byte health frame (``54 1F …``)."""
    if len(data) != HEALTH_PACKET_SIZE:
        raise ProtocolError(f"health packet must be {HEALTH_PACKET_SIZE} bytes, got {len(data)}")
    raw = bytes(data)
    if raw[0] != HEADER or raw[1] != HEALTH_INFO_MARK:
        raise ProtocolError(f"bad health header: {raw[0]:02X} {raw[1]:02X}")
    if crc8(raw[:-1]) != raw[-1]:
        raise ProtocolError("health packet CRC mismatch")
    error_code = struct.unpack_from("<H", raw, 2)[0]
    mode_val = raw[4]
    try:
        mode = HealthMode(mode_val)
    except ValueError:
        mode = HealthMode.ERROR
    flags = tuple(name for bit, name in HEALTH_FLAGS.items() if error_code & (1 << bit))
    return HealthInfo(error_code=error_code, mode=mode, flags=flags, raw=raw)


def parse_command_response(data: bytes | bytearray | memoryview) -> Tuple[int, bytes]:
    """Parse an 8-byte LD14P command echo/response.

    Returns:
        ``(command_code, payload_4_bytes)``
    """
    if len(data) != COMMAND_SIZE:
        raise ProtocolError(f"command response must be {COMMAND_SIZE} bytes, got {len(data)}")
    raw = bytes(data)
    if raw[0] != HEADER or raw[2] != 0x04:
        raise ProtocolError("bad command response framing")
    if crc8(raw[:-1]) != raw[-1]:
        raise ProtocolError("command response CRC mismatch")
    return raw[1], raw[3:7]


Frame = Union[ScanPacket, HealthInfo]


class FrameStreamParser:
    """Incremental byte-stream demultiplexer for scan / health / command frames.

    LD14 is scan-only (unidirectional). LD14P may interleave scan packets,
    ~1 Hz health packets, and command echoes on the same UART.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, chunk: bytes | bytearray | memoryview) -> List[Frame]:
        """Append bytes and return any complete frames decoded."""
        if not chunk:
            return []
        self._buf.extend(chunk)
        out: List[Frame] = []
        while True:
            frame = self._try_pop()
            if frame is None:
                break
            out.append(frame)
        return out

    def _try_pop(self) -> Optional[Frame]:
        buf = self._buf
        # Resync to next 0x54.
        while buf and buf[0] != HEADER:
            del buf[0]
        if len(buf) < 2:
            return None

        mark = buf[1]
        if mark == SCAN_VER_LEN:
            need = SCAN_PACKET_SIZE
            if len(buf) < need:
                return None
            candidate = bytes(buf[:need])
            del buf[:need]
            try:
                return parse_scan_packet(candidate)
            except ProtocolError:
                # Bad CRC / corrupt: drop the header byte and resync.
                self._buf = bytearray(candidate[1:]) + buf
                return None

        if mark == HEALTH_INFO_MARK:
            need = HEALTH_PACKET_SIZE
            if len(buf) < need:
                return None
            candidate = bytes(buf[:need])
            del buf[:need]
            try:
                return parse_health_packet(candidate)
            except ProtocolError:
                self._buf = bytearray(candidate[1:]) + buf
                return None

        # LD14P command responses use cmd in byte1 (0xA0–0xA3), length 0x04.
        if mark in (
            MotorCommand.START,
            MotorCommand.STOP,
            MotorCommand.SET_SPEED,
            MotorCommand.QUERY_SPEED,
        ):
            need = COMMAND_SIZE
            if len(buf) < need:
                return None
            candidate = bytes(buf[:need])
            del buf[:need]
            try:
                parse_command_response(candidate)
            except ProtocolError:
                self._buf = bytearray(candidate[1:]) + buf
                return None
            # Caller that needs raw command ACKs can subclass; default drops them.
            return None

        # Unknown after 0x54 — skip one byte.
        del buf[0]
        return None


def iter_scan_packets(frames: Iterable[Frame]) -> Iterable[ScanPacket]:
    for frame in frames:
        if isinstance(frame, ScanPacket):
            yield frame
