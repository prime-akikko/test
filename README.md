# LDS14 Mini LiDAR API

Python UART client for the **LDRobot LD14 / LD14P** family, commonly sold as the **LDS14 Mini LiDAR 360° Scanning Module**.

## Install

```bash
python -m pip install -e ".[serial]"
# or
python -m pip install -r requirements.txt
python -m pip install -e .
```

## Quick start

```python
from lds14 import Lds14, Model

# LD14P: bidirectional UART @ 230400, software start/stop
with Lds14("/dev/ttyUSB0", model=Model.LD14P) as lidar:
    lidar.start()
    packet = lidar.poll_scan(timeout=2.0)
    if packet:
        for p in packet.points:
            print(p.angle_deg, p.distance_mm, p.intensity)
    lidar.stop()

# Classic LD14: Tx-only stream @ 115200 (motor via PWM / power)
with Lds14("/dev/ttyUSB0", model=Model.LD14) as lidar:
    for pkt in lidar.iter_packets(max_packets=5):
        print(pkt.speed_hz, len(pkt.points))
```

CLI example:

```bash
python examples/stream_scan.py /dev/ttyUSB0 --model ld14p --packets 10
python examples/stream_scan.py COM3 --model ld14
```

## Protocol (short)

| Model  | Baud   | Direction     | Start/stop                          |
|--------|--------|---------------|-------------------------------------|
| LD14   | 115200 | Tx → host     | Power/PWM; no UART commands         |
| LD14P  | 230400 | Full duplex   | UART `0xA0` / `0xA1` (+ speed/health) |

Scan frame (both): **47 bytes**, `0x54 0x2C`, 12 × `(uint16 distance_mm, uint8 intensity)`, start/end angle in 0.01°, speed deg/s, timestamp ms, CRC-8 poly `0x4D`.

Assumptions and sources are noted in code comments and in the Project writeup.

## Tests

```bash
python -m pip install -e ".[dev]"
python -m unittest discover -s tests -v
```

## Layout

- `src/lds14/` — library (`Lds14`, `parse_scan_packet`, motor helpers)
- `examples/stream_scan.py` — serial smoke example
- `tests/` — protocol CRC / parse tests (no hardware required)
