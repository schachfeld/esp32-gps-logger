#!/usr/bin/env python3
"""Convert raw UBX-NAV-PVT logs from the GPS logger to GPX 1.1."""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import os
import pathlib
import struct
import sys
import tempfile
from typing import BinaryIO, Iterator, Optional, TextIO
from xml.sax.saxutils import escape


UBX_SYNC = b"\xb5\x62"
NAV_CLASS = 0x01
NAV_PVT_ID = 0x07
NAV_PVT_MINIMUM_LENGTH = 92
MAX_LOGGED_PAYLOAD_LENGTH = 4096


@dataclasses.dataclass
class ParseStats:
    valid_messages: int = 0
    checksum_errors: int = 0
    oversized_headers: int = 0
    discarded_bytes: int = 0
    truncated_tail: bool = False


@dataclasses.dataclass(frozen=True)
class UbxMessage:
    message_class: int
    message_id: int
    payload: bytes


@dataclasses.dataclass(frozen=True)
class TrackPoint:
    timestamp: dt.datetime
    latitude_e7: int
    longitude_e7: int
    altitude_msl_mm: int
    horizontal_accuracy_mm: int
    vertical_accuracy_mm: int
    ground_speed_mm_s: int
    heading_e5: int
    position_dop_centi: int
    satellites: int
    fix_type: int

    @property
    def fix_label(self) -> str:
        return "2d" if self.fix_type == 2 else "3d"


def ubx_checksum(message_body: bytes) -> bytes:
    """Return UBX Fletcher checksum bytes for class through payload."""
    checksum_a = 0
    checksum_b = 0
    for byte in message_body:
        checksum_a = (checksum_a + byte) & 0xFF
        checksum_b = (checksum_b + checksum_a) & 0xFF
    return bytes((checksum_a, checksum_b))


def iter_ubx_messages(
    source: BinaryIO,
    stats: Optional[ParseStats] = None,
    chunk_size: int = 64 * 1024,
) -> Iterator[UbxMessage]:
    """Yield checksum-valid UBX packets without loading the whole track into memory."""
    if stats is None:
        stats = ParseStats()
    if chunk_size < 8:
        raise ValueError("chunk_size must be at least 8 bytes")

    buffer = bytearray()
    end_of_file = False
    while not end_of_file or buffer:
        if not end_of_file:
            chunk = source.read(chunk_size)
            if chunk:
                buffer.extend(chunk)
            else:
                end_of_file = True

        cursor = 0
        waiting_for_more = False
        while cursor < len(buffer):
            sync_at = buffer.find(UBX_SYNC, cursor)
            if sync_at < 0:
                preserve = 1 if not end_of_file and buffer[-1:] == UBX_SYNC[:1] else 0
                stats.discarded_bytes += len(buffer) - cursor - preserve
                cursor = len(buffer) - preserve
                waiting_for_more = preserve == 1
                break

            stats.discarded_bytes += sync_at - cursor
            if len(buffer) - sync_at < 6:
                if end_of_file:
                    stats.truncated_tail = True
                    stats.discarded_bytes += len(buffer) - sync_at
                    cursor = len(buffer)
                else:
                    cursor = sync_at
                    waiting_for_more = True
                break

            payload_length = buffer[sync_at + 4] | (buffer[sync_at + 5] << 8)
            if payload_length > MAX_LOGGED_PAYLOAD_LENGTH:
                stats.oversized_headers += 1
                cursor = sync_at + 1
                continue

            packet_length = payload_length + 8
            if len(buffer) - sync_at < packet_length:
                if end_of_file:
                    stats.truncated_tail = True
                    cursor = sync_at + 1
                    continue
                cursor = sync_at
                waiting_for_more = True
                break

            packet = bytes(buffer[sync_at : sync_at + packet_length])
            if ubx_checksum(packet[2:-2]) != packet[-2:]:
                stats.checksum_errors += 1
                cursor = sync_at + 1
                continue

            stats.valid_messages += 1
            yield UbxMessage(packet[2], packet[3], packet[6:-2])
            cursor = sync_at + packet_length

        if cursor:
            del buffer[:cursor]
        if end_of_file and waiting_for_more:
            stats.truncated_tail = True
            stats.discarded_bytes += len(buffer)
            buffer.clear()
        elif end_of_file and not cursor and buffer:
            stats.discarded_bytes += len(buffer)
            buffer.clear()


def parse_nav_pvt(payload: bytes) -> Optional[TrackPoint]:
    """Decode one UBX-NAV-PVT payload, returning only a usable position fix."""
    if len(payload) < NAV_PVT_MINIMUM_LENGTH:
        return None

    valid = payload[11]
    fix_type = payload[20]
    flags = payload[21]
    flags3 = payload[78]
    time_is_resolved = valid & 0x07 == 0x07
    position_is_usable = fix_type in (2, 3, 4) and flags & 0x01 and not flags3 & 0x01
    if not time_is_resolved or not position_is_usable:
        return None

    year = struct.unpack_from("<H", payload, 4)[0]
    month, day, hour, minute, second = payload[6:11]
    nanoseconds = struct.unpack_from("<i", payload, 16)[0]
    try:
        minute_start = dt.datetime(year, month, day, hour, minute, tzinfo=dt.timezone.utc)
        timestamp = minute_start + dt.timedelta(
            seconds=second, microseconds=round(nanoseconds / 1000)
        )
    except (OverflowError, ValueError):
        return None

    return TrackPoint(
        timestamp=timestamp,
        longitude_e7=struct.unpack_from("<i", payload, 24)[0],
        latitude_e7=struct.unpack_from("<i", payload, 28)[0],
        altitude_msl_mm=struct.unpack_from("<i", payload, 36)[0],
        horizontal_accuracy_mm=struct.unpack_from("<I", payload, 40)[0],
        vertical_accuracy_mm=struct.unpack_from("<I", payload, 44)[0],
        ground_speed_mm_s=struct.unpack_from("<i", payload, 60)[0],
        heading_e5=struct.unpack_from("<i", payload, 64)[0],
        position_dop_centi=struct.unpack_from("<H", payload, 76)[0],
        satellites=payload[23],
        fix_type=fix_type,
    )


def iter_track_points(source: BinaryIO, stats: Optional[ParseStats] = None) -> Iterator[TrackPoint]:
    for message in iter_ubx_messages(source, stats):
        if message.message_class == NAV_CLASS and message.message_id == NAV_PVT_ID:
            point = parse_nav_pvt(message.payload)
            if point is not None:
                yield point


def format_timestamp(timestamp: dt.datetime) -> str:
    return timestamp.isoformat(timespec="milliseconds").replace("+00:00", "Z")


def write_gpx(points: Iterator[TrackPoint], destination: TextIO, track_name: str) -> int:
    destination.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    destination.write(
        '<gpx version="1.1" creator="esp32-gps-logger" '
        'xmlns="http://www.topografix.com/GPX/1/1" '
        'xmlns:ubx="urn:esp32-gps-logger:ubx" '
        'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
        'xsi:schemaLocation="http://www.topografix.com/GPX/1/1 '
        'http://www.topografix.com/GPX/1/1/gpx.xsd">\n'
    )
    destination.write("  <trk>\n")
    destination.write(f"    <name>{escape(track_name)}</name>\n")
    destination.write("    <trkseg>\n")

    count = 0
    for point in points:
        destination.write(
            f'      <trkpt lat="{point.latitude_e7 / 10_000_000:.7f}" '
            f'lon="{point.longitude_e7 / 10_000_000:.7f}">\n'
        )
        destination.write(f"        <ele>{point.altitude_msl_mm / 1000:.3f}</ele>\n")
        destination.write(f"        <time>{format_timestamp(point.timestamp)}</time>\n")
        destination.write(f"        <fix>{point.fix_label}</fix>\n")
        destination.write(f"        <sat>{point.satellites}</sat>\n")
        destination.write(f"        <pdop>{point.position_dop_centi / 100:.2f}</pdop>\n")
        destination.write("        <extensions>\n")
        destination.write(
            f"          <ubx:hacc>{point.horizontal_accuracy_mm / 1000:.3f}</ubx:hacc>\n"
        )
        destination.write(
            f"          <ubx:vacc>{point.vertical_accuracy_mm / 1000:.3f}</ubx:vacc>\n"
        )
        destination.write(
            f"          <ubx:speed>{point.ground_speed_mm_s / 1000:.3f}</ubx:speed>\n"
        )
        destination.write(f"          <ubx:course>{point.heading_e5 / 100_000:.5f}</ubx:course>\n")
        destination.write("        </extensions>\n")
        destination.write("      </trkpt>\n")
        count += 1

    destination.write("    </trkseg>\n")
    destination.write("  </trk>\n")
    destination.write("</gpx>\n")
    return count


def convert_file(input_path: pathlib.Path, output_path: pathlib.Path) -> tuple[int, ParseStats]:
    if input_path.resolve() == output_path.resolve():
        raise ValueError("input and output paths must differ")

    stats = ParseStats()
    temporary_path: Optional[pathlib.Path] = None
    try:
        with input_path.open("rb") as source, tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            dir=output_path.parent,
            prefix=f".{output_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as output:
            temporary_path = pathlib.Path(output.name)
            point_count = write_gpx(iter_track_points(source, stats), output, input_path.stem)
        os.replace(temporary_path, output_path)
    except BaseException:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise
    return point_count, stats


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert a raw UBX-NAV-PVT track from the ESP32 logger to GPX 1.1."
    )
    parser.add_argument("input", type=pathlib.Path, help="input .ubx track")
    parser.add_argument(
        "output",
        type=pathlib.Path,
        nargs="?",
        help="output .gpx path (default: input name with .gpx)",
    )
    return parser


def main(arguments: Optional[list[str]] = None) -> int:
    options = build_argument_parser().parse_args(arguments)
    output_path = options.output or options.input.with_suffix(".gpx")
    try:
        point_count, stats = convert_file(options.input, output_path)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    print(
        f"Wrote {point_count} points to {output_path} "
        f"({stats.valid_messages} valid UBX packets, "
        f"{stats.checksum_errors} checksum errors)."
    )
    if stats.truncated_tail:
        print("Note: ignored an incomplete packet at the end of the log.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
