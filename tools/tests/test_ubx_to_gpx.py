import io
import pathlib
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as element_tree


sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))

import ubx_to_gpx  # noqa: E402


def make_nav_pvt(
    *,
    latitude_e7=-338_651_430,
    longitude_e7=1_512_099_000,
    fix_type=3,
    valid=0x07,
    flags=0x01,
    flags3=0,
    second=5,
    nanoseconds=250_000_000,
):
    payload = bytearray(92)
    struct.pack_into("<H", payload, 4, 2026)
    payload[6:11] = bytes((9, 17, 12, 34, second))
    payload[11] = valid
    struct.pack_into("<i", payload, 16, nanoseconds)
    payload[20] = fix_type
    payload[21] = flags
    payload[23] = 12
    struct.pack_into("<i", payload, 24, longitude_e7)
    struct.pack_into("<i", payload, 28, latitude_e7)
    struct.pack_into("<i", payload, 36, 42_125)
    struct.pack_into("<I", payload, 40, 1_250)
    struct.pack_into("<I", payload, 44, 2_500)
    struct.pack_into("<i", payload, 60, 3_750)
    struct.pack_into("<i", payload, 64, 12_345_678)
    struct.pack_into("<H", payload, 76, 123)
    payload[78] = flags3
    body = bytes((ubx_to_gpx.NAV_CLASS, ubx_to_gpx.NAV_PVT_ID))
    body += struct.pack("<H", len(payload)) + payload
    return ubx_to_gpx.UBX_SYNC + body + ubx_to_gpx.ubx_checksum(body)


class UbxToGpxTests(unittest.TestCase):
    def test_parser_resynchronizes_after_junk_and_bad_checksum(self):
        bad_packet = bytearray(make_nav_pvt())
        bad_packet[-1] ^= 0xFF
        stats = ubx_to_gpx.ParseStats()
        data = b"junk" + bytes(bad_packet) + b"noise" + make_nav_pvt() + b"\xb5\x62\x01"

        messages = list(ubx_to_gpx.iter_ubx_messages(io.BytesIO(data), stats, chunk_size=17))

        self.assertEqual(1, len(messages))
        self.assertEqual(1, stats.checksum_errors)
        self.assertTrue(stats.truncated_tail)

    def test_nav_pvt_decodes_signed_coordinates_and_leap_second(self):
        packet = make_nav_pvt(second=60, nanoseconds=-250_000_000)
        message = next(ubx_to_gpx.iter_ubx_messages(io.BytesIO(packet)))

        point = ubx_to_gpx.parse_nav_pvt(message.payload)

        self.assertIsNotNone(point)
        self.assertEqual(-338_651_430, point.latitude_e7)
        self.assertEqual(1_512_099_000, point.longitude_e7)
        self.assertEqual("2026-09-17T12:34:59.750Z", ubx_to_gpx.format_timestamp(point.timestamp))
        self.assertEqual("3d", point.fix_label)

    def test_nav_pvt_rejects_unresolved_or_invalid_positions(self):
        variants = (
            make_nav_pvt(valid=0x03),
            make_nav_pvt(fix_type=1),
            make_nav_pvt(flags=0),
            make_nav_pvt(flags3=1),
        )
        for packet in variants:
            with self.subTest(packet=packet[20:30]):
                message = next(ubx_to_gpx.iter_ubx_messages(io.BytesIO(packet)))
                self.assertIsNone(ubx_to_gpx.parse_nav_pvt(message.payload))

    def test_gpx_is_well_formed_and_uses_schema_element_order(self):
        message = next(ubx_to_gpx.iter_ubx_messages(io.BytesIO(make_nav_pvt())))
        point = ubx_to_gpx.parse_nav_pvt(message.payload)
        output = io.StringIO()

        count = ubx_to_gpx.write_gpx(iter((point,)), output, "morning & evening")
        xml = output.getvalue()
        root = element_tree.fromstring(xml)
        namespace = {"g": "http://www.topografix.com/GPX/1/1"}
        track_point = root.find(".//g:trkpt", namespace)

        self.assertEqual(1, count)
        self.assertEqual("-33.8651430", track_point.attrib["lat"])
        self.assertEqual("151.2099000", track_point.attrib["lon"])
        self.assertIn("<name>morning &amp; evening</name>", xml)
        self.assertLess(xml.index("<ele>"), xml.index("<time>"))
        self.assertLess(xml.index("<time>"), xml.index("<fix>"))
        self.assertLess(xml.index("<fix>"), xml.index("<sat>"))

    def test_convert_file_reports_packets_and_writes_default_data(self):
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / "S00000042.ubx"
            destination = pathlib.Path(directory) / "track.gpx"
            source.write_bytes(make_nav_pvt(latitude_e7=525_200_080, longitude_e7=134_049_540))

            count, stats = ubx_to_gpx.convert_file(source, destination)

            self.assertEqual(1, count)
            self.assertEqual(1, stats.valid_messages)
            self.assertIn('lat="52.5200080" lon="13.4049540"', destination.read_text())


if __name__ == "__main__":
    unittest.main()
