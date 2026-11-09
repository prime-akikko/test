"""High-level serial client for LD14 / LD14P (LDS14 Mini) modules."""

from __future__ import annotations

import time
from enum import Enum
from typing import Iterator, List, Optional, Union

from .protocol import (
    COMMAND_SIZE,
    Frame,
    FrameStreamParser,
    HealthInfo,
    MotorCommand,
    ScanPacket,
    parse_command_response,
    query_speed_command,
    set_speed_command,
    start_motor_command,
    stop_motor_command,
)

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - exercised only without pyserial
    serial = None  # type: ignore


class Model(str, Enum):
    """Hardware profile affecting baud and whether UART motor commands exist.

    - ``LD14``: unidirectional UART @ 115200; motor via PWM pin (not serial).
      Scanning begins after power-on with no host commands (LD14 Manual V1.2).
    - ``LD14P``: bidirectional UART @ 230400; start/stop/speed/health over serial
      (LD14P Manual V0.2 / LD14P Datasheet).

    Retail "LDS14 Mini LiDAR 360°" modules are typically one of these LDRobot
    parts. Prefer ``LD14P`` when the cable exposes RX and software start/stop
    is required; use ``LD14`` for Tx-only adapters at 115200.
    """

    LD14 = "ld14"
    LD14P = "ld14p"


DEFAULT_BAUD = {
    Model.LD14: 115_200,
    Model.LD14P: 230_400,
}


class Lds14:
    """Connect, optionally start/stop the motor, and stream scan packets.

    Example::

        from lds14 import Lds14, Model

        with Lds14("/dev/ttyUSB0", model=Model.LD14P) as lidar:
            lidar.start()
            for packet in lidar.iter_packets(max_packets=20):
                print(packet.speed_hz, packet.points[0])
            lidar.stop()
    """

    def __init__(
        self,
        port: str,
        *,
        model: Model = Model.LD14P,
        baudrate: Optional[int] = None,
        timeout: float = 1.0,
        serial_kwargs: Optional[dict] = None,
    ) -> None:
        if serial is None:
            raise ImportError(
                "pyserial is required to open a serial port. "
                "Install with: pip install 'lds14[serial]' or pip install pyserial"
            )
        self.port = port
        self.model = Model(model)
        self.baudrate = baudrate if baudrate is not None else DEFAULT_BAUD[self.model]
        self.timeout = timeout
        self._serial_kwargs = dict(serial_kwargs or {})
        self._ser: Optional["serial.Serial"] = None
        self._parser = FrameStreamParser()
        self._last_health: Optional[HealthInfo] = None
        self._last_scan: Optional[ScanPacket] = None

    def open(self) -> "Lds14":
        if self._ser is not None and self._ser.is_open:
            return self
        self._parser.reset()
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            **self._serial_kwargs,
        )
        return self

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None
        self._parser.reset()

    def __enter__(self) -> "Lds14":
        return self.open()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def last_health(self) -> Optional[HealthInfo]:
        return self._last_health

    @property
    def last_scan(self) -> Optional[ScanPacket]:
        return self._last_scan

    def _require_serial(self) -> "serial.Serial":
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("serial port is not open; call open() first")
        return self._ser

    def _supports_motor_uart(self) -> bool:
        return self.model == Model.LD14P

    def start(self) -> None:
        """Start scanning / motor.

        LD14P: sends ``0xA0`` start command over UART.
        LD14: no-op for UART (module streams after power-on; motor is PWM).
        """
        if not self._supports_motor_uart():
            return
        self._send_command(start_motor_command(), expect_cmd=MotorCommand.START)

    def stop(self) -> None:
        """Stop motor (LD14P only). On LD14 this is a documented no-op."""
        if not self._supports_motor_uart():
            return
        self._send_command(stop_motor_command(), expect_cmd=MotorCommand.STOP)

    def set_speed_hz(self, hz: float) -> None:
        """Set LD14P target scan rate in Hz (valid range typically 2–8)."""
        if not self._supports_motor_uart():
            raise RuntimeError("UART speed control requires Model.LD14P with RX wired")
        speed_deg_s = int(round(hz * 360.0))
        self._send_command(
            set_speed_command(speed_deg_s),
            expect_cmd=MotorCommand.SET_SPEED,
        )

    def query_speed_deg_s(self) -> int:
        """Query LD14P target speed (degrees/second)."""
        if not self._supports_motor_uart():
            raise RuntimeError("UART speed query requires Model.LD14P with RX wired")
        payload = self._send_command(
            query_speed_command(),
            expect_cmd=MotorCommand.QUERY_SPEED,
        )
        return int.from_bytes(payload[:2], "little")

    def _send_command(self, frame: bytes, *, expect_cmd: int, wait_s: float = 0.5) -> bytes:
        ser = self._require_serial()
        ser.write(frame)
        ser.flush()
        deadline = time.monotonic() + wait_s
        leftover = bytearray()
        while time.monotonic() < deadline:
            waiting = ser.in_waiting
            chunk = ser.read(waiting or 1)
            if not chunk:
                continue
            leftover.extend(chunk)
            # Search for matching 8-byte response while retaining other bytes for parser.
            i = 0
            while i + COMMAND_SIZE <= len(leftover):
                window = bytes(leftover[i : i + COMMAND_SIZE])
                if window[0] == 0x54 and window[1] == expect_cmd and window[2] == 0x04:
                    try:
                        cmd, payload = parse_command_response(window)
                    except Exception:
                        i += 1
                        continue
                    # Feed non-response bytes into the stream parser.
                    before = bytes(leftover[:i])
                    after = bytes(leftover[i + COMMAND_SIZE :])
                    self._ingest(before + after)
                    return payload
                i += 1
        # Timed out: still ingest whatever we read so scan sync is not lost.
        self._ingest(bytes(leftover))
        raise TimeoutError(f"no response for command 0x{expect_cmd:02X}")

    def _ingest(self, data: bytes) -> List[Frame]:
        frames = self._parser.feed(data)
        for frame in frames:
            if isinstance(frame, ScanPacket):
                self._last_scan = frame
            elif isinstance(frame, HealthInfo):
                self._last_health = frame
        return frames

    def read(self, size: int = 256) -> List[Frame]:
        """Read available serial bytes once and return newly decoded frames."""
        ser = self._require_serial()
        waiting = ser.in_waiting
        chunk = ser.read(waiting or size)
        return self._ingest(chunk)

    def poll_scan(self, *, timeout: Optional[float] = None) -> Optional[ScanPacket]:
        """Block until the next scan packet or timeout; return ``None`` on timeout."""
        ser = self._require_serial()
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                return None
            frames = self.read()
            for frame in frames:
                if isinstance(frame, ScanPacket):
                    return frame
            if not frames:
                # Force a blocking read of at least one byte when idle.
                chunk = ser.read(1)
                if chunk:
                    for frame in self._ingest(chunk):
                        if isinstance(frame, ScanPacket):
                            return frame
                elif deadline is None:
                    # Non-blocking empty read with infinite wait would spin; small sleep.
                    time.sleep(0.001)

    def iter_packets(
        self,
        *,
        max_packets: Optional[int] = None,
        include_health: bool = False,
    ) -> Iterator[Union[ScanPacket, HealthInfo]]:
        """Yield scan packets (and optionally health frames) until stopped.

        Stops after ``max_packets`` *scan* packets when that limit is set.
        """
        scans = 0
        while max_packets is None or scans < max_packets:
            for frame in self.read():
                if isinstance(frame, ScanPacket):
                    scans += 1
                    yield frame
                    if max_packets is not None and scans >= max_packets:
                        return
                elif include_health and isinstance(frame, HealthInfo):
                    yield frame
            if not self._require_serial().in_waiting:
                # Brief block to avoid busy-spin when the OS buffer is empty.
                chunk = self._require_serial().read(1)
                if chunk:
                    for frame in self._ingest(chunk):
                        if isinstance(frame, ScanPacket):
                            scans += 1
                            yield frame
                            if max_packets is not None and scans >= max_packets:
                                return
                        elif include_health and isinstance(frame, HealthInfo):
                            yield frame

    def health(self) -> Optional[HealthInfo]:
        """Return the most recently observed LD14P health packet, if any.

        Health frames are emitted about once per second on LD14P. Call
        ``iter_packets(include_health=True)`` or ``read()`` to refresh.
        Classic LD14 has no UART health channel.
        """
        return self._last_health
