"""Protocol edge cases and end-to-end Linux pseudo-terminal checks."""

import csv
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import termios
import time
import unittest

from imu_test import Parser, checksum, decode_payload


# Fixed hand-checkable packet: TID=1, temperature=25 C; CK_A=D5, CK_B=B6.
TEMP_FRAME = bytes.fromhex("59 53 01 00 04 01 02 c4 09 d5 b6")


def packet(payload, tid=7):
    body = struct.pack("<HB", tid, len(payload)) + payload
    # Independent weighted-sum form of the two checksum bytes.
    tail = bytes((sum(body) & 255,
                  sum((len(body) - i) * b for i, b in enumerate(body)) & 255))
    return b"\x59\x53" + body + tail


def imu_packet():
    return packet(b"\x10\x0c" + struct.pack("<3i", -1000000, 0, 9806650) +
                  b"\x20\x0c" + struct.pack("<3i", 0, -2000000, 3000000) +
                  b"\x40\x0c" + struct.pack("<3i", 10000000, -20000000, 30000000))


class ProtocolTest(unittest.TestCase):
    def test_fixed_temperature_frame(self):
        self.assertEqual(checksum(TEMP_FRAME[2:-2]), TEMP_FRAME[-2:])
        self.assertEqual(Parser().feed(TEMP_FRAME), [(1, {"temp_C": 25.0})])

    def test_bytewise_and_back_to_back(self):
        parser = Parser()
        result = []
        data = b"garbage\x59" + imu_packet() * 3
        for value in data:
            result.extend(parser.feed(bytes((value,))))
        self.assertEqual(len(result), 3)
        self.assertEqual(result[0][1]["acc_m_s2"], (-1.0, 0.0, 9.80665))
        self.assertEqual(result[0][1]["gyro_deg_s"], (0.0, -2.0, 3.0))
        self.assertEqual(result[0][1]["rpy_deg"], (-20.0, 10.0, 30.0))
        self.assertEqual(len(parser.buffer), 0)
        self.assertEqual(len(Parser().feed(imu_packet() * 3)), 3)

    def test_corrupt_checksum_and_recovery(self):
        damaged = bytearray(imu_packet())
        damaged[-1] ^= 1
        parser = Parser()
        result = parser.feed(damaged + imu_packet())
        self.assertEqual(len(result), 1)
        self.assertEqual(parser.bad_checksum, 1)

    def test_unknown_fields_and_malformed_lengths(self):
        self.assertEqual(decode_payload(b"\xf0\x03abc\x01\x02\x9c\xff"),
                         {"temp_C": -1.0})
        for payload in (b"\x10", b"\x10\x0c\x00", b"\x10\x01\x00"):
            parser = Parser()
            result = parser.feed(packet(payload) + imu_packet())
            self.assertEqual(len(result), 1)
            self.assertEqual(parser.bad_payload, 1)

    def test_buffer_bounded_without_frames(self):
        parser = Parser()
        parser.feed(b"noise" * 10000 + b"\x59")
        self.assertEqual(parser.buffer, b"\x59")
        self.assertEqual(len(parser.feed(imu_packet()[1:])), 1)

    def test_quaternion(self):
        data = b"\x41\x10" + struct.pack("<4i", 1000000, 0, -1, 0)
        self.assertEqual(decode_payload(data)["quat_wxyz"], (1.0, 0.0, -0.000001, 0.0))


class SerialTest(unittest.TestCase):
    def probe(self, data=None, raw=None, csv_path=None, interrupt=False):
        output_dir = tempfile.TemporaryDirectory()
        if csv_path is None:
            csv_path = Path(output_dir.name) / "imu.csv"
        master, slave = pty.openpty()
        before = termios.tcgetattr(slave)
        command = [sys.executable, "-B", str(Path(__file__).with_name("imu_test.py")),
                   "--port", os.ttyname(slave), "--duration", "5" if interrupt else "0.3",
                   "--csv", str(csv_path)]
        if raw is not None:
            command.extend(["--raw", str(raw)])
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True)
        try:
            ready, _, _ = select.select([proc.stdout], [], [], 5.0)
            self.assertTrue(ready, "probe did not open the pseudo-terminal")
            first = proc.stdout.readline()
            self.assertTrue(first.startswith("OPEN:"), first)
            active = termios.tcgetattr(slave)
            self.assertEqual(active[4:6], [termios.B460800, termios.B460800])
            if data is not None:
                os.write(master, data)
            if interrupt:
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    if len(csv_path.read_text().splitlines()) >= 6:
                        break
                    time.sleep(0.01)
                else:
                    self.fail("CSV rows were not flushed during capture")
                proc.send_signal(signal.SIGINT)
            stdout, stderr = proc.communicate(timeout=5)
            # Raw receive-only mode must never echo or transmit payload bytes.
            self.assertFalse(select.select([master], [], [], 0)[0])
            self.assertEqual(termios.tcgetattr(slave), before)
            return proc.returncode, first + stdout, stderr
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.communicate()
            proc.stdout.close()
            proc.stderr.close()
            os.close(master)
            os.close(slave)
            output_dir.cleanup()

    def test_receive_pass_and_binary_capture(self):
        data = imu_packet() * 5
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "raw.bin"
            code, output, errors = self.probe(data, raw)
            self.assertEqual(code, 0, output + errors)
            self.assertIn("imu_frames=5", output)
            self.assertIn("PASS:", output)
            self.assertEqual(raw.read_bytes(), data)

    def test_csv_saves_every_frame_and_leaves_missing_fields_empty(self):
        damaged = bytearray(imu_packet())
        damaged[-1] ^= 1
        data = damaged + imu_packet() * 5 + TEMP_FRAME
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "nested" / "imu.csv"
            code, output, errors = self.probe(data, csv_path=csv_path)
            self.assertEqual(code, 0, output + errors)
            with csv_path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 6)
            self.assertIn("(6 rows)", output)
            self.assertEqual(rows[0]["tid"], "7")
            self.assertEqual(float(rows[0]["acc_x_m_s2"]), -1.0)
            self.assertEqual(float(rows[0]["roll_deg"]), -20.0)
            self.assertGreater(float(rows[0]["time_unix_s"]), 0)
            self.assertGreaterEqual(float(rows[0]["elapsed_s"]), 0)
            self.assertEqual(rows[0]["temp_C"], "")
            self.assertEqual(rows[5]["acc_x_m_s2"], "")
            self.assertEqual(rows[5]["quat_w"], "")
            self.assertEqual(float(rows[5]["temp_C"]), 25.0)

    def test_csv_rows_survive_interrupt(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "imu.csv"
            code, output, errors = self.probe(imu_packet() * 5, csv_path=csv_path,
                                              interrupt=True)
            self.assertEqual(code, 130, output + errors)
            with csv_path.open(newline="") as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 5)

    def test_existing_csv_is_not_overwritten(self):
        master, slave = pty.openpty()
        before = termios.tcgetattr(slave)
        try:
            with tempfile.TemporaryDirectory() as directory:
                csv_path = Path(directory) / "imu.csv"
                csv_path.write_text("previous capture\n")
                result = subprocess.run(
                    [sys.executable, "-B", str(Path(__file__).with_name("imu_test.py")),
                     "--port", os.ttyname(slave), "--csv", str(csv_path)],
                    capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 6, result.stdout + result.stderr)
                self.assertEqual(csv_path.read_text(), "previous capture\n")
                self.assertEqual(termios.tcgetattr(slave), before)
        finally:
            os.close(master)
            os.close(slave)

    def test_no_bytes_is_not_driver_diagnosis(self):
        code, output, errors = self.probe()
        self.assertEqual(code, 3, output + errors)
        self.assertIn("bytes=0", output)

    def test_invalid_wire_data(self):
        code, output, errors = self.probe(b"garbage" * 10)
        self.assertEqual(code, 4, output + errors)

    def test_temperature_only_does_not_pass_imu_check(self):
        code, output, errors = self.probe(TEMP_FRAME * 5)
        self.assertEqual(code, 5, output + errors)


if __name__ == "__main__":
    unittest.main()
