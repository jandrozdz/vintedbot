#!/usr/bin/env python3
"""
TrikiReader - Python port of https://github.com/AND-Y0/TrikiReader (C# / WPF)

Connects to the Triki BLE motion-controller (Żabka gaming device) via the
Nordic UART Service, decodes 14-byte IMU frames, applies sensor fusion, and
streams orientation to the console and optionally to a CSV file.

Usage:
    pip install bleak
    python triki_reader.py
    python triki_reader.py --output data.csv --filter madgwick

Run `python triki_reader.py --help` for all options.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import signal
import struct
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Iterator, Optional

# ──────────────────────────────────────────────────────────────────────────────
# BLE UUIDs (Nordic UART Service)
# ──────────────────────────────────────────────────────────────────────────────

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write
NUS_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify

# ──────────────────────────────────────────────────────────────────────────────
# Constants / defaults (match AppOptions.Default in C#)
# ──────────────────────────────────────────────────────────────────────────────

FRAME_LENGTH       = 14
FRAME_HEADER       = (0x22, 0x00)

DEFAULT_DEVICE_NAME    = "Triki"
DEFAULT_START_CMD      = bytes.fromhex("201000D007680003")
DEFAULT_GYRO_SCALE     = 131.0   # LSB / (°/s)
DEFAULT_ACCEL_SCALE    = 2048.0  # LSB / g
DEFAULT_STARTUP_DISCARD = 20
DEFAULT_SCAN_TIMEOUT   = 30.0
DEFAULT_SETTLE_DELAY   = 3.0
DEFAULT_CONSOLE_EVERY  = 20


# ──────────────────────────────────────────────────────────────────────────────
# Data types
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class ImuSample:
    frame_index: int
    timestamp:   datetime
    gyro_x:  float
    gyro_y:  float
    gyro_z:  float
    accel_x: float
    accel_y: float
    accel_z: float
    raw_gyro_x:  int
    raw_gyro_y:  int
    raw_gyro_z:  int
    raw_accel_x: int
    raw_accel_y: int
    raw_accel_z: int

    @staticmethod
    def from_frame(frame: bytes, index: int,
                   gyro_scale: float, accel_scale: float,
                   ts: Optional[datetime] = None) -> "ImuSample":
        if ts is None:
            ts = datetime.now(timezone.utc)
        # frame layout: [0x22,0x00] gx gy gz ax ay az  (each 2 bytes LE int16)
        raw_gx, raw_gy, raw_gz = struct.unpack_from("<hhh", frame, 2)
        raw_ax, raw_ay, raw_az = struct.unpack_from("<hhh", frame, 8)
        return ImuSample(
            frame_index=index,
            timestamp=ts,
            gyro_x=raw_gx / gyro_scale,
            gyro_y=raw_gy / gyro_scale,
            gyro_z=raw_gz / gyro_scale,
            accel_x=raw_ax / accel_scale,
            accel_y=raw_ay / accel_scale,
            accel_z=raw_az / accel_scale,
            raw_gyro_x=raw_gx, raw_gyro_y=raw_gy, raw_gyro_z=raw_gz,
            raw_accel_x=raw_ax, raw_accel_y=raw_ay, raw_accel_z=raw_az,
        )


@dataclass
class ImuOrientation:
    pitch: float = 0.0
    roll:  float = 0.0
    yaw:   float = 0.0


# ──────────────────────────────────────────────────────────────────────────────
# Frame parser  (mirrors FrameParser.cs)
# ──────────────────────────────────────────────────────────────────────────────

class FrameParser:
    """Accumulates raw BLE bytes and extracts fixed-length 14-byte IMU frames."""

    def __init__(self) -> None:
        self._buf: bytearray = bytearray()
        self.dropped_bytes: int = 0

    def push(self, data: bytes) -> Iterator[bytes]:
        self._buf.extend(data)

        while True:
            idx = self._find_header()

            if idx < 0:
                if self._buf:
                    keep_last = self._buf[-1] == 0x22
                    drop = len(self._buf) - (1 if keep_last else 0)
                    self.dropped_bytes += drop
                    if keep_last:
                        last = self._buf[-1]
                        self._buf.clear()
                        self._buf.append(last)
                    else:
                        self._buf.clear()
                return

            if idx > 0:
                self.dropped_bytes += idx
                del self._buf[:idx]

            if len(self._buf) < FRAME_LENGTH:
                return

            yield bytes(self._buf[:FRAME_LENGTH])
            del self._buf[:FRAME_LENGTH]

    def _find_header(self) -> int:
        for i in range(len(self._buf) - 1):
            if self._buf[i] == FRAME_HEADER[0] and self._buf[i + 1] == FRAME_HEADER[1]:
                return i
        return -1


# ──────────────────────────────────────────────────────────────────────────────
# Madgwick AHRS  (mirrors MadgwickAHRS.cs)
# ──────────────────────────────────────────────────────────────────────────────

class MadgwickAHRS:
    """
    Madgwick's IMU/AHRS algorithm.
    Reference: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    """

    def __init__(self, beta: float = 0.1) -> None:
        self.beta = beta
        self.q: list[float] = [1.0, 0.0, 0.0, 0.0]  # [w, x, y, z]
        self._last_ts: Optional[datetime] = None

    def update_from_sample(self, s: ImuSample) -> None:
        if self._last_ts is None:
            self._last_ts = s.timestamp
            return
        dt = max((s.timestamp - self._last_ts).total_seconds(), 0.0)
        self._last_ts = s.timestamp
        self.update(s.gyro_x, s.gyro_y, s.gyro_z,
                    s.accel_x, s.accel_y, s.accel_z, dt)

    def update(self, gx: float, gy: float, gz: float,
               ax: float, ay: float, az: float, dt: float) -> None:
        if dt <= 0.0:
            return
        q1, q2, q3, q4 = self.q

        _2q1 = 2.0 * q1; _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3; _2q4 = 2.0 * q4
        _4q1 = 4.0 * q1; _4q2 = 4.0 * q2; _4q3 = 4.0 * q3
        _8q2 = 8.0 * q2; _8q3 = 8.0 * q3
        q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3; q4q4 = q4*q4

        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0.0:
            return
        ax /= norm; ay /= norm; az /= norm

        s1 = _4q1*q3q3 + _2q3*ax + _4q1*q2q2 - _2q2*ay
        s2 = (_4q2*q4q4 - _2q4*ax + 4.0*q1q1*q2 - _2q1*ay
              - _4q2 + _8q2*q2q2 + _8q2*q3q3 + _4q2*az)
        s3 = (4.0*q1q1*q3 + _2q1*ax + _4q3*q4q4 - _2q4*ay
              - _4q3 + _8q3*q2q2 + _8q3*q3q3 + _4q3*az)
        s4 = 4.0*q2q2*q4 - _2q2*ax + 4.0*q3q3*q4 - _2q3*ay

        norm = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if norm > 0.0:
            s1 /= norm; s2 /= norm; s3 /= norm; s4 /= norm

        qDot1 = 0.5*(-q2*gx - q3*gy - q4*gz) - self.beta*s1
        qDot2 = 0.5*( q1*gx + q3*gz - q4*gy) - self.beta*s2
        qDot3 = 0.5*( q1*gy - q2*gz + q4*gx) - self.beta*s3
        qDot4 = 0.5*( q1*gz + q2*gy - q3*gx) - self.beta*s4

        q1 += qDot1*dt; q2 += qDot2*dt
        q3 += qDot3*dt; q4 += qDot4*dt

        norm = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
        self.q = [q1/norm, q2/norm, q3/norm, q4/norm]

    def reset(self) -> None:
        self.q = [1.0, 0.0, 0.0, 0.0]
        self._last_ts = None

    @property
    def euler_degrees(self) -> ImuOrientation:
        w, x, y, z = self.q
        pitch = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
        roll  = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
        yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
        return ImuOrientation(pitch, roll, yaw)


# ──────────────────────────────────────────────────────────────────────────────
# Complementary orientation filter  (mirrors ImuOrientationFilter.cs)
# ──────────────────────────────────────────────────────────────────────────────

class ImuOrientationFilter:
    """
    Complementary filter fusing accelerometer (tilt) and gyroscope (rate).
    alpha=0.98 means 98 % trust in the gyro integral, 2 % in the accelerometer.
    """

    def __init__(self, alpha: float = 0.98, yaw_gain: float = 2.5) -> None:
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("alpha must be in [0, 1]")
        if yaw_gain <= 0.0:
            raise ValueError("yaw_gain must be > 0")
        self._alpha = alpha
        self._yaw_gain = yaw_gain
        self._last_ts: Optional[datetime] = None
        self.pitch = 0.0
        self.roll  = 0.0
        self.yaw   = 0.0

    def update(self, sample: ImuSample) -> ImuOrientation:
        accel_pitch = math.degrees(math.atan2(
            sample.accel_x,
            math.sqrt(sample.accel_y**2 + sample.accel_z**2)))
        accel_roll = math.degrees(math.atan2(sample.accel_y, -sample.accel_z))

        if self._last_ts is None:
            self.pitch = accel_pitch
            self.roll  = accel_roll
            self.yaw   = 0.0
            self._last_ts = sample.timestamp
            return ImuOrientation(self.pitch, self.roll, self.yaw)

        dt = max((sample.timestamp - self._last_ts).total_seconds(), 0.0)
        self._last_ts = sample.timestamp

        gyro_pitch = self.pitch + sample.gyro_y * dt
        gyro_roll  = self.roll  + sample.gyro_x * dt
        self.yaw  += sample.gyro_z * self._yaw_gain * dt

        self.pitch = self._alpha * gyro_pitch + (1.0 - self._alpha) * accel_pitch
        self.roll  = self._alpha * gyro_roll  + (1.0 - self._alpha) * accel_roll
        return ImuOrientation(self.pitch, self.roll, self.yaw)

    def reset(self) -> None:
        self.pitch = self.roll = self.yaw = 0.0
        self._last_ts = None


# ──────────────────────────────────────────────────────────────────────────────
# Statistics  (mirrors ImuStats.cs)
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class ImuStats:
    notification_count:       int   = 0
    parsed_frame_count:       int   = 0
    discarded_startup_count:  int   = 0
    written_sample_count:     int   = 0
    dropped_bytes:            int   = 0
    last_gap_ms:              float = 0.0
    max_gap_ms:               float = 0.0
    _last_ts: Optional[datetime] = field(default=None, repr=False)

    def notification_received(self, ts: Optional[datetime] = None) -> None:
        if ts is None:
            ts = datetime.now(timezone.utc)
        if self._last_ts is not None:
            gap = max(0.0, (ts - self._last_ts).total_seconds() * 1000.0)
            self.last_gap_ms = gap
            self.max_gap_ms  = max(self.max_gap_ms, gap)
        self._last_ts = ts
        self.notification_count += 1


# ──────────────────────────────────────────────────────────────────────────────
# Main reader
# ──────────────────────────────────────────────────────────────────────────────

class TrikiReader:
    def __init__(
        self,
        device_name:      str   = DEFAULT_DEVICE_NAME,
        gyro_scale:       float = DEFAULT_GYRO_SCALE,
        accel_scale:      float = DEFAULT_ACCEL_SCALE,
        startup_discard:  int   = DEFAULT_STARTUP_DISCARD,
        scan_timeout:     float = DEFAULT_SCAN_TIMEOUT,
        settle_delay:     float = DEFAULT_SETTLE_DELAY,
        start_cmd:        bytes = DEFAULT_START_CMD,
        output_csv:       Optional[str] = None,
        filter_mode:      str   = "complementary",
        console_every:    int   = DEFAULT_CONSOLE_EVERY,
    ) -> None:
        self._device_name     = device_name
        self._gyro_scale      = gyro_scale
        self._accel_scale     = accel_scale
        self._startup_discard = startup_discard
        self._scan_timeout    = scan_timeout
        self._settle_delay    = settle_delay
        self._start_cmd       = start_cmd
        self._output_csv      = output_csv
        self._console_every   = console_every

        self._stats      = ImuStats()
        self._parser     = FrameParser()
        self._frame_idx  = 0
        self._discarded  = 0
        self._stop_event = asyncio.Event()

        if filter_mode == "madgwick":
            self._filter: MadgwickAHRS | ImuOrientationFilter = MadgwickAHRS(beta=0.1)
        else:
            self._filter = ImuOrientationFilter(alpha=0.98, yaw_gain=2.5)

        self._csv_file   = None
        self._csv_writer = None

    # ── public ────────────────────────────────────────────────────────────────

    async def run(self) -> None:
        try:
            from bleak import BleakScanner, BleakClient
        except ImportError:
            print("ERROR: 'bleak' is not installed. Run: pip install bleak", file=sys.stderr)
            sys.exit(1)

        self._log(f"Scanning for '{self._device_name}' (timeout={self._scan_timeout}s)...")
        device = await BleakScanner.find_device_by_filter(
            lambda d, _: bool(d.name and self._device_name.lower() in d.name.lower()),
            timeout=self._scan_timeout,
        )
        if device is None:
            self._log(
                "No matching BLE device found. "
                "Press the button on Triki to wake it, then run again."
            )
            return

        self._log(f"Found: {device.name} [{device.address}]")

        if self._output_csv:
            self._csv_file = open(self._output_csv, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "frame_index", "timestamp_utc",
                "gyro_x", "gyro_y", "gyro_z",
                "accel_x", "accel_y", "accel_z",
                "pitch_deg", "roll_deg", "yaw_deg",
            ])

        try:
            async with BleakClient(device) as client:
                self._log(f"Connected to {device.name}")

                if self._settle_delay > 0:
                    self._log(
                        f"Place Triki flat and keep it still. "
                        f"Starting stream in {self._settle_delay:.0f}s..."
                    )
                    await asyncio.sleep(self._settle_delay)

                self._log("Subscribing to NUS TX notifications...")
                await client.start_notify(NUS_TX_UUID, self._on_notification)

                if self._start_cmd:
                    self._log(f"Writing start command: {self._start_cmd.hex()}")
                    await client.write_gatt_char(NUS_RX_UUID, self._start_cmd, response=False)

                self._log("Streaming IMU data — press Ctrl+C to stop.")
                await self._stop_event.wait()
                await client.stop_notify(NUS_TX_UUID)

        finally:
            if self._csv_file:
                self._csv_file.close()
            print()  # newline after \r status line
            self._log(
                f"Session ended — "
                f"notifications={self._stats.notification_count}, "
                f"frames={self._stats.parsed_frame_count}, "
                f"samples={self._stats.written_sample_count}, "
                f"dropped_bytes={self._stats.dropped_bytes}, "
                f"max_gap_ms={self._stats.max_gap_ms:.1f}"
            )

    def stop(self) -> None:
        self._stop_event.set()

    # ── internal ──────────────────────────────────────────────────────────────

    def _on_notification(self, _sender: object, data: bytearray) -> None:
        ts = datetime.now(timezone.utc)
        self._stats.notification_received(ts)

        for frame in self._parser.push(bytes(data)):
            self._stats.parsed_frame_count += 1
            self._stats.dropped_bytes = self._parser.dropped_bytes

            if self._discarded < self._startup_discard:
                self._discarded += 1
                self._stats.discarded_startup_count += 1
                continue

            sample = ImuSample.from_frame(
                frame, self._frame_idx,
                self._gyro_scale, self._accel_scale, ts,
            )
            self._frame_idx += 1
            self._stats.written_sample_count += 1

            if isinstance(self._filter, MadgwickAHRS):
                self._filter.update_from_sample(sample)
                orientation = self._filter.euler_degrees
            else:
                orientation = self._filter.update(sample)

            if self._csv_writer is not None:
                self._csv_writer.writerow([
                    sample.frame_index,
                    sample.timestamp.isoformat(),
                    f"{sample.gyro_x:.4f}",  f"{sample.gyro_y:.4f}",  f"{sample.gyro_z:.4f}",
                    f"{sample.accel_x:.4f}", f"{sample.accel_y:.4f}", f"{sample.accel_z:.4f}",
                    f"{orientation.pitch:.2f}", f"{orientation.roll:.2f}", f"{orientation.yaw:.2f}",
                ])

            if self._frame_idx % self._console_every == 0:
                print(
                    f"\r[{self._frame_idx:6d}] "
                    f"Gyro  ({sample.gyro_x:+7.2f} {sample.gyro_y:+7.2f} {sample.gyro_z:+7.2f}) °/s  "
                    f"Accel ({sample.accel_x:+5.2f} {sample.accel_y:+5.2f} {sample.accel_z:+5.2f}) g  "
                    f"P={orientation.pitch:+6.1f}° R={orientation.roll:+6.1f}° Y={orientation.yaw:+7.1f}°",
                    end="", flush=True,
                )

    @staticmethod
    def _log(msg: str) -> None:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────

def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Read IMU data from Triki BLE device (Python port of TrikiReader)")
    p.add_argument("--device", default=DEFAULT_DEVICE_NAME,
                   help="BLE device name substring to scan for (default: Triki)")
    p.add_argument("--gyro-scale", type=float, default=DEFAULT_GYRO_SCALE,
                   help="Gyroscope LSB per °/s (default: 131.0)")
    p.add_argument("--accel-scale", type=float, default=DEFAULT_ACCEL_SCALE,
                   help="Accelerometer LSB per g (default: 2048.0)")
    p.add_argument("--startup-discard", type=int, default=DEFAULT_STARTUP_DISCARD,
                   help="Frames to discard at startup (default: 20)")
    p.add_argument("--scan-timeout", type=float, default=DEFAULT_SCAN_TIMEOUT,
                   help="BLE scan timeout in seconds (default: 30)")
    p.add_argument("--settle-delay", type=float, default=DEFAULT_SETTLE_DELAY,
                   help="Seconds to wait before sending start command (default: 3)")
    p.add_argument("--start-cmd", default=DEFAULT_START_CMD.hex(),
                   help="Hex-encoded start command written to NUS RX "
                        "(default: 201000D007680003, set to '' to skip)")
    p.add_argument("--output", "-o", default=None, metavar="FILE",
                   help="Write samples to this CSV file")
    p.add_argument("--filter", choices=["complementary", "madgwick"],
                   default="complementary",
                   help="Orientation filter (default: complementary)")
    p.add_argument("--console-every", type=int, default=DEFAULT_CONSOLE_EVERY,
                   help="Print a status line every N samples (default: 20)")
    return p


def main() -> None:
    args = _build_arg_parser().parse_args()

    start_cmd = bytes.fromhex(args.start_cmd) if args.start_cmd else b""

    reader = TrikiReader(
        device_name=args.device,
        gyro_scale=args.gyro_scale,
        accel_scale=args.accel_scale,
        startup_discard=args.startup_discard,
        scan_timeout=args.scan_timeout,
        settle_delay=args.settle_delay,
        start_cmd=start_cmd,
        output_csv=args.output,
        filter_mode=args.filter,
        console_every=args.console_every,
    )

    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, reader.stop)
        except (NotImplementedError, OSError):
            pass

    try:
        loop.run_until_complete(reader.run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
