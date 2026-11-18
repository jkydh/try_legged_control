"""Exercise the actual C++ Linux reader through a PTY; no ROS or IMU required."""
import csv
import io
import math
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


BINARY = None


def packet(payload, tid=42):
    body = struct.pack('<HB', tid, len(payload)) + payload
    return b'\x59\x53' + body + bytes((sum(body) & 255,
        sum((len(body) - i) * value for i, value in enumerate(body)) & 255))


ACC = b'\x10\x0c' + struct.pack('<3i', -1000000, 0, 9806650)
GYRO = b'\x20\x0c' + struct.pack('<3i', 180000000, -90000000, 0)
QUAT = b'\x41\x10' + struct.pack('<4i', 707107, 0, 0, 707107)
EULER = b'\x40\x0c' + struct.pack('<3i', 10000000, -20000000, 30000000)
TEMP = b'\x01\x02' + struct.pack('<h', -1234)
FULL = packet(ACC + GYRO + QUAT + EULER + TEMP)


class ReaderTest(unittest.TestCase):
    def read_stream(self, data=b'', extra=(), interrupt=False, temperature_stream=False, fragmented=False):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / 'h30.csv'
            master, slave = pty.openpty()
            inherited = termios.tcgetattr(slave)
            inherited[0] |= termios.IXON | termios.IXOFF | termios.IXANY
            termios.tcsetattr(slave, termios.TCSANOW, inherited)
            before = termios.tcgetattr(slave)
            proc = subprocess.Popen([str(BINARY), '--port', os.ttyname(slave),
                '--duration', '0' if interrupt else '0.35', '--startup-timeout', '0.15',
                '--data-timeout', '0.12', '--csv', str(csv_path), *extra],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                self.assertTrue(select.select([proc.stdout], [], [], 5)[0])
                first = proc.stdout.readline()
                self.assertTrue(first.startswith('OPEN:'), first + proc.stderr.read() if proc.poll() is not None else first)
                active = termios.tcgetattr(slave)
                self.assertEqual(active[4:6], [termios.B460800, termios.B460800])
                self.assertFalse(active[0] & (termios.IXON | termios.IXOFF | termios.IXANY))
                self.assertFalse(active[2] & (termios.PARENB | termios.CSTOPB | termios.CRTSCTS))
                self.assertEqual(active[2] & termios.CSIZE, termios.CS8)
                if fragmented:
                    for value in data:
                        os.write(master, bytes((value,)))
                        time.sleep(0.0001)
                elif data:
                    os.write(master, data)
                if interrupt:
                    deadline = time.monotonic() + 2
                    while len(csv_path.read_text().splitlines()) < 2:
                        self.assertLess(time.monotonic(), deadline)
                        time.sleep(0.005)
                    proc.send_signal(signal.SIGINT)
                elif temperature_stream:
                    end = time.monotonic() + 0.25
                    while time.monotonic() < end and proc.poll() is None:
                        os.write(master, packet(TEMP))
                        time.sleep(0.01)
                stdout, stderr = proc.communicate(timeout=5)
                self.assertEqual(termios.tcgetattr(slave), before)
                self.assertFalse(select.select([master], [], [], 0)[0], 'reader transmitted or echoed UART bytes')
                return proc.returncode, first + stdout + stderr, list(csv.DictReader(io.StringIO(csv_path.read_text())))
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.communicate()
                proc.stdout.close()
                proc.stderr.close()
                os.close(master)
                os.close(slave)

    def test_full_frame_units_euler_order_temperature_and_all_csv_rows(self):
        code, output, rows = self.read_stream(FULL * 5, extra=('--duration', '0.08'))
        self.assertEqual(code, 0, output)
        self.assertEqual(len(rows), 5)
        row = rows[0]
        self.assertAlmostEqual(float(row['acc_z_m_s2']), 9.80665)
        self.assertAlmostEqual(float(row['gyro_x_rad_s']), math.pi)
        self.assertAlmostEqual(float(row['gyro_y_rad_s']), -math.pi / 2)
        self.assertAlmostEqual(float(row['quat_z']), math.sqrt(0.5))
        self.assertAlmostEqual(float(row['quat_w']), math.sqrt(0.5))
        self.assertAlmostEqual(float(row['roll_rad']), math.radians(-20))
        self.assertAlmostEqual(float(row['pitch_rad']), math.radians(10))
        self.assertAlmostEqual(float(row['yaw_rad']), math.radians(30))
        self.assertAlmostEqual(float(row['temp_C']), -12.34)
        self.assertEqual(row['complete'], '1')

    def test_corrupt_checksum_noise_fragmentation_and_unknown_tlv(self):
        bad = bytearray(FULL)
        bad[-1] ^= 1
        code, output, rows = self.read_stream(b'noise\x59' + bad + packet(ACC + GYRO + QUAT + b'\xf0\x03abc'),
            extra=('--duration', '0.08'), fragmented=True)
        self.assertEqual(code, 0, output)
        self.assertEqual(len(rows), 1)
        self.assertIn('checksum_errors=1', output)

    def test_partial_frames_do_not_reuse_previous_fields(self):
        code, output, rows = self.read_stream(FULL + packet(EULER), extra=('--duration', '0.08'))
        self.assertEqual(code, 0, output)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[1]['complete'], '0')
        for field in ('quat_w', 'acc_z_m_s2', 'gyro_x_rad_s', 'temp_C'):
            self.assertEqual(rows[1][field], '')
        self.assertNotEqual(rows[1]['roll_rad'], '')

    def test_strict_mode_rejects_partial_frames_instead_of_merging_them(self):
        code, output, rows = self.read_stream(packet(ACC) + packet(GYRO) + packet(QUAT), extra=('--require-complete',))
        self.assertEqual(code, 1, output)
        self.assertEqual(rows, [])
        self.assertIn('payload_errors=3', output)

    def test_strict_mode_accepts_complete_frame(self):
        code, output, rows = self.read_stream(FULL, extra=('--require-complete', '--duration', '0.08'))
        self.assertEqual(code, 0, output)
        self.assertEqual(len(rows), 1)

    def test_bad_tlv_and_invalid_quaternion_rejected(self):
        invalid = packet(b'\x10\x01\x00') + packet(ACC + GYRO + b'\x41\x10' + bytes(16))
        code, output, rows = self.read_stream(invalid + FULL, extra=('--duration', '0.08'))
        self.assertEqual(code, 0, output)
        self.assertEqual(len(rows), 1)
        self.assertIn('payload_errors=2', output)

    def test_startup_without_bytes_fails(self):
        code, output, rows = self.read_stream()
        self.assertEqual(code, 1, output)
        self.assertIn('bytes=0', output)
        self.assertEqual(rows, [])

    def test_bad_wire_data_fails(self):
        code, output, rows = self.read_stream(b'garbage' * 20)
        self.assertEqual(code, 1, output)
        self.assertIn('No accepted IMU frame', output)

    def test_temperature_only_is_not_imu_success(self):
        code, output, rows = self.read_stream(packet(TEMP), temperature_stream=True)
        self.assertEqual(code, 1, output)
        self.assertIn('imu_frames=0', output)
        self.assertTrue(rows)

    def test_stream_loss_is_detected_even_if_temperature_continues(self):
        code, output, rows = self.read_stream(FULL, temperature_stream=True)
        self.assertEqual(code, 1, output)
        self.assertIn('IMU stream timed out', output)

    def test_interrupt_keeps_csv_and_restores_uart(self):
        code, output, rows = self.read_stream(FULL * 3, interrupt=True)
        self.assertEqual(code, 130, output)
        self.assertEqual(len(rows), 3)

    def test_invalid_arguments(self):
        for args in (('--duration', 'nan'), ('--baud', '460800.5'), ('--data-timeout', '0'), ('--unknown', 'x')):
            result = subprocess.run([str(BINARY), *args], capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)

    def test_existing_csv_not_overwritten(self):
        master, slave = pty.openpty()
        before = termios.tcgetattr(slave)
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'previous.csv'
                path.write_text('preserve this\n')
                result = subprocess.run([str(BINARY), '--port', os.ttyname(slave), '--csv', str(path)],
                    capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 1)
                self.assertEqual(path.read_text(), 'preserve this\n')
                self.assertEqual(termios.tcgetattr(slave), before)
        finally:
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    BINARY = Path(sys.argv.pop(1)).resolve()
    unittest.main()
