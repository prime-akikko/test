"""Protocol unit tests using the LD14 Development Manual V1.2 example frame."""

from __future__ import annotations

import struct
import unittest

from lds14.crc import crc8
from lds14.protocol import (
    FrameStreamParser,
    HealthInfo,
    ScanPacket,
    build_command,
    parse_health_packet,
    parse_scan_packet,
    query_speed_command,
    set_speed_command,
    start_motor_command,
    stop_motor_command,
)

# Hex dump from LD14 Development Manual V1.2 §3.3 (47 bytes).
MANUAL_SCAN_HEX = (
    "54 2C 68 08 AB 7E "
    "E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9 "
    "CD 00 E4 CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 "
    "BE 82 3A 1A 50"
)


def _manual_frame() -> bytes:
    return bytes.fromhex(MANUAL_SCAN_HEX.replace(" ", ""))


class CrcTests(unittest.TestCase):
    def test_manual_scan_crc(self) -> None:
        frame = _manual_frame()
        self.assertEqual(len(frame), 47)
        self.assertEqual(crc8(frame[:-1]), frame[-1])
        self.assertEqual(frame[-1], 0x50)

    def test_reference_motor_commands(self) -> None:
        self.assertEqual(start_motor_command(), bytes.fromhex("54A004000000005E"))
        self.assertEqual(stop_motor_command(), bytes.fromhex("54A104000000004A"))
        self.assertEqual(set_speed_command(2160), bytes.fromhex("54A20470080000A1"))
        self.assertEqual(query_speed_command(), bytes.fromhex("54A3040000000062"))
        # 7 Hz = 2520 deg/s from LD14P manual.
        self.assertEqual(set_speed_command(2520), bytes.fromhex("54A204D809000016"))


class ScanParseTests(unittest.TestCase):
    def test_manual_example_fields(self) -> None:
        pkt = parse_scan_packet(_manual_frame())
        self.assertIsInstance(pkt, ScanPacket)
        self.assertEqual(pkt.speed_deg_s, 0x0868)
        self.assertAlmostEqual(pkt.start_angle_deg, 324.27, places=2)
        self.assertAlmostEqual(pkt.end_angle_deg, 334.70, places=2)
        self.assertEqual(pkt.timestamp_ms, 0x1A3A)
        self.assertEqual(len(pkt.points), 12)
        self.assertEqual(pkt.points[0].distance_mm, 0x00E0)
        self.assertEqual(pkt.points[0].intensity, 0xE4)
        self.assertEqual(pkt.points[1].distance_mm, 0x00DC)
        self.assertEqual(pkt.points[1].intensity, 0xE2)
        # First angle equals start; last equals end (interpolation endpoints).
        self.assertAlmostEqual(pkt.points[0].angle_deg, pkt.start_angle_deg, places=5)
        self.assertAlmostEqual(pkt.points[-1].angle_deg, pkt.end_angle_deg, places=5)

    def test_angle_wrap(self) -> None:
        # Build a minimal valid packet crossing 0°.
        body = bytearray(47)
        body[0] = 0x54
        body[1] = 0x2C
        struct.pack_into("<HH", body, 2, 2160, 35900)  # start 359.00°
        for i in range(12):
            struct.pack_into("<HB", body, 6 + 3 * i, 100 + i, 50)
        struct.pack_into("<HH", body, 42, 100, 1234)  # end 1.00°
        body[-1] = crc8(body[:-1])
        pkt = parse_scan_packet(bytes(body))
        self.assertGreater(pkt.points[1].angle_deg, pkt.points[0].angle_deg % 360 or 0)
        # Midpoint should be near 0° after wrap.
        mid = pkt.points[6].angle_deg
        self.assertTrue(mid < 10 or mid > 350)


class HealthParseTests(unittest.TestCase):
    def test_ok_health(self) -> None:
        body = bytearray([0x54, 0x1F, 0x00, 0x00, 0x00, 0x00])
        body[-1] = crc8(body[:-1])
        info = parse_health_packet(bytes(body))
        self.assertTrue(info.ok)
        self.assertEqual(info.flags, ())

    def test_motor_flag(self) -> None:
        err = 1 << 5
        body = bytearray([0x54, 0x1F, err & 0xFF, (err >> 8) & 0xFF, 0x02, 0x00])
        body[-1] = crc8(body[:-1])
        info = parse_health_packet(bytes(body))
        self.assertIn("motor_or_locked_rotor", info.flags)
        self.assertFalse(info.ok)


class StreamParserTests(unittest.TestCase):
    def test_feed_split_scan(self) -> None:
        frame = _manual_frame()
        parser = FrameStreamParser()
        self.assertEqual(parser.feed(frame[:10]), [])
        out = parser.feed(frame[10:])
        self.assertEqual(len(out), 1)
        self.assertIsInstance(out[0], ScanPacket)

    def test_interleaved_health(self) -> None:
        scan = _manual_frame()
        health = bytearray([0x54, 0x1F, 0x00, 0x00, 0x00, 0x00])
        health[-1] = crc8(health[:-1])
        parser = FrameStreamParser()
        frames = parser.feed(bytes(health) + scan)
        self.assertEqual(len(frames), 2)
        self.assertIsInstance(frames[0], HealthInfo)
        self.assertIsInstance(frames[1], ScanPacket)

    def test_build_command_crc(self) -> None:
        frame = build_command(0xA0)
        self.assertEqual(len(frame), 8)
        self.assertEqual(crc8(frame[:-1]), frame[-1])


if __name__ == "__main__":
    unittest.main()
