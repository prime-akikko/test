#!/usr/bin/env python3
"""Stream a few LDS14 / LD14(P) scan packets from a USB-UART adapter.

Usage:
  python examples/stream_scan.py /dev/ttyUSB0
  python examples/stream_scan.py COM3 --model ld14
  python examples/stream_scan.py /dev/ttyUSB0 --model ld14p --hz 6 --packets 5
"""

from __future__ import annotations

import argparse
import sys

from lds14 import Lds14, Model, ScanPacket


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0 or COM3")
    parser.add_argument(
        "--model",
        choices=[m.value for m in Model],
        default=Model.LD14P.value,
        help="ld14 @ 115200 (Tx-only stream); ld14p @ 230400 (UART motor control)",
    )
    parser.add_argument("--baud", type=int, default=None, help="Override default baud")
    parser.add_argument("--packets", type=int, default=10, help="Scan packets to print")
    parser.add_argument("--hz", type=float, default=None, help="LD14P target scan rate")
    parser.add_argument("--no-start", action="store_true", help="Skip UART start command")
    args = parser.parse_args(argv)

    model = Model(args.model)
    with Lds14(args.port, model=model, baudrate=args.baud) as lidar:
        if not args.no_start:
            lidar.start()
        if args.hz is not None:
            lidar.set_speed_hz(args.hz)

        print(
            f"Listening on {args.port} @ {lidar.baudrate} ({model.value}); "
            f"Ctrl-C to stop",
            file=sys.stderr,
        )
        try:
            for i, frame in enumerate(lidar.iter_packets(max_packets=args.packets), 1):
                assert isinstance(frame, ScanPacket)
                valid = sum(1 for p in frame.points if p.valid)
                p0 = frame.points[0]
                print(
                    f"[{i:03d}] {frame.start_angle_deg:6.2f}→{frame.end_angle_deg:6.2f}°  "
                    f"{frame.speed_hz:4.2f} Hz  valid={valid}/12  "
                    f"p0={p0.distance_mm}mm@{p0.angle_deg:.2f}° I={p0.intensity}"
                )
        except KeyboardInterrupt:
            print("Interrupted", file=sys.stderr)
        finally:
            if model is Model.LD14P and not args.no_start:
                try:
                    lidar.stop()
                except Exception as exc:  # noqa: BLE001 — best-effort shutdown
                    print(f"stop failed: {exc}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
