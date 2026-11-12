#!/usr/bin/env python3
"""Linux H30 mini receive-only probe, using Python 3 standard library.

Wire format reference: Seeed-Projects/WHEELTEC-H30-IMU, Yesense_Decode.py.
This implementation does not send configuration or calibration commands.
"""

import argparse
import contextlib
import csv
from datetime import datetime, timezone
import errno
import fcntl
import math
import os
from pathlib import Path
import select
import struct
import sys
import termios
import time
import tty


CSV_VECTORS = {
    "acc_m_s2": ("acc_x_m_s2", "acc_y_m_s2", "acc_z_m_s2"),
    "gyro_deg_s": ("gyro_x_deg_s", "gyro_y_deg_s", "gyro_z_deg_s"),
    "rpy_deg": ("roll_deg", "pitch_deg", "yaw_deg"),
    "quat_wxyz": ("quat_w", "quat_x", "quat_y", "quat_z"),
}
CSV_COLUMNS = ["time_unix_s", "elapsed_s", "tid"] + [
    column for columns in CSV_VECTORS.values() for column in columns
] + ["temp_C"]


def csv_record(tid, fields, wall_time_ns, elapsed):
    seconds, nanos = divmod(wall_time_ns, 1_000_000_000)
    row = {"time_unix_s": "%d.%09d" % (seconds, nanos),
           "elapsed_s": "%.9f" % elapsed, "tid": tid}
    for key, columns in CSV_VECTORS.items():
        if key in fields:
            row.update(zip(columns, fields[key]))
    if "temp_C" in fields:
        row["temp_C"] = fields["temp_C"]
    return row


def checksum(data):
    a = b = 0
    for value in data:
        a = (a + value) & 255
        b = (b + a) & 255
    return bytes((a, b))


def decode_payload(payload):
    """Decode one TLV payload; unknown fields are skipped by their length."""
    fields = {}
    pos = 0
    vectors = {0x10: "acc_m_s2", 0x20: "gyro_deg_s"}
    while pos < len(payload):
        if pos + 2 > len(payload):
            raise ValueError("truncated TLV header")
        kind, size = payload[pos:pos + 2]
        pos += 2
        if pos + size > len(payload):
            raise ValueError("truncated TLV value")
        value = payload[pos:pos + size]
        pos += size
        expected = {0x01: 2, 0x10: 12, 0x20: 12, 0x40: 12, 0x41: 16}
        if kind in expected and size != expected[kind]:
            raise ValueError("invalid length for data ID 0x%02x" % kind)
        if kind in vectors:
            fields[vectors[kind]] = tuple(v / 1e6 for v in struct.unpack("<3i", value))
        elif kind == 0x40:
            pitch, roll, yaw = struct.unpack("<3i", value)
            fields["rpy_deg"] = (roll / 1e6, pitch / 1e6, yaw / 1e6)
        elif kind == 0x41:
            fields["quat_wxyz"] = tuple(v / 1e6 for v in struct.unpack("<4i", value))
        elif kind == 0x01:
            fields["temp_C"] = struct.unpack("<h", value)[0] / 100.0
    return fields


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.valid = 0
        self.bad_checksum = 0
        self.bad_payload = 0
        self.discarded = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(b"\x59\x53")
            if start < 0:
                keep = int(self.buffer.endswith(b"\x59"))
                self.discarded += len(self.buffer) - keep
                self.buffer[:] = self.buffer[-1:] if keep else b""
                break
            self.discarded += start
            del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            length = self.buffer[4] + 7
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            if checksum(frame[2:-2]) != frame[-2:]:
                self.bad_checksum += 1
                del self.buffer[0]  # Search again, including inside damaged frames.
                continue
            del self.buffer[:length]
            try:
                fields = decode_payload(frame[5:-2])
            except ValueError:
                self.bad_payload += 1
                continue
            self.valid += 1
            frames.append((struct.unpack_from("<H", frame, 2)[0], fields))
        return frames


def ports():
    devices = []
    for entry in sorted(Path("/sys/class/tty").glob("*")):
        if not entry.name.startswith(("ttyACM", "ttyUSB", "ttyCH343USB")):
            continue
        device = (entry / "device").resolve()
        vendor = product = "?"
        for parent in (device,) + tuple(device.parents):
            if (parent / "idVendor").exists():
                vendor = (parent / "idVendor").read_text().strip()
                product = (parent / "idProduct").read_text().strip()
                break
        driver = (device / "driver").resolve().name
        devices.append(("/dev/" + entry.name, vendor, product, driver))
    return devices


@contextlib.contextmanager
def serial_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    saved = None
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        saved = termios.tcgetattr(fd)
        tty.setraw(fd, termios.TCSANOW)
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB |
                      termios.CRTSCTS | termios.HUPCL)
        attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = getattr(termios, "B%d" % baud)
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        yield fd
    finally:
        if saved is not None:
            with contextlib.suppress(OSError, termios.error):
                termios.tcsetattr(fd, termios.TCSANOW, saved)
        with contextlib.suppress(OSError):
            fcntl.ioctl(fd, termios.TIOCNXCL)
        os.close(fd)


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return number


def run(args, path):
    parser = Parser()
    total = imu_frames = 0
    prefix = bytearray()
    latest = {}
    seen = set()
    next_print = 0.0
    with contextlib.ExitStack() as stack:
        fd = stack.enter_context(serial_port(path, args.baud))
        raw = stack.enter_context(open(args.raw, "xb")) if args.raw else None
        csv_path = args.csv
        if csv_path is None:
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            csv_path = Path(__file__).resolve().parent / "artifacts" / ("imu_" + stamp + ".csv")
        csv_path = csv_path.resolve()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = stack.enter_context(csv_path.open("x", encoding="utf-8", newline=""))
        csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
        csv_writer.writeheader()
        csv_file.flush()
        rows = 0
        print("OPEN: %s, %d baud, 8N1, no flow control; receiving %.1f s" %
              (path, args.baud, args.duration), flush=True)
        print("CSV: %s" % csv_path, flush=True)
        started = time.monotonic()
        deadline = started + args.duration
        while time.monotonic() < deadline:
            ready, _, _ = select.select([fd], [], [], max(0, min(0.2, deadline - time.monotonic())))
            if not ready:
                continue
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not data:
                raise OSError(errno.EIO, "串口已断开或返回 EOF")
            received_ns = time.time_ns()
            received_elapsed = time.monotonic() - started
            total += len(data)
            prefix.extend(data[:max(0, 64 - len(prefix))])
            if raw:
                raw.write(data)
            for tid, fields in parser.feed(data):
                csv_writer.writerow(csv_record(tid, fields, received_ns, received_elapsed))
                rows += 1
                if set(fields) & {"acc_m_s2", "gyro_deg_s", "rpy_deg", "quat_wxyz"}:
                    imu_frames += 1
                seen.update(fields)
                latest = fields  # Show only this frame, avoiding stale field values.
                now = time.monotonic()
                if now >= next_print:
                    values = []
                    for key, value in latest.items():
                        rendered = (", ".join("%.6f" % v for v in value)
                                    if isinstance(value, tuple) else "%.2f" % value)
                        values.append("%s=(%s)" % (key, rendered))
                    print("tid=%d  %s" % (tid, "  ".join(values) or "no supported IMU fields"), flush=True)
                    next_print = now + 0.2
            csv_file.flush()
        elapsed = time.monotonic() - started
    print("SAVED: %s (%d rows)" % (csv_path, rows))
    print("SUMMARY: bytes=%d valid_frames=%d imu_frames=%d checksum_errors=%d "
          "payload_errors=%d discarded_bytes=%d buffered_bytes=%d avg_frames_hz=%.1f" %
          (total, parser.valid, imu_frames, parser.bad_checksum, parser.bad_payload,
           parser.discarded, len(parser.buffer), parser.valid / elapsed))
    print("Fields seen: " + (", ".join(sorted(seen)) or "none"))
    if total == 0:
        print("FAIL: 串口已打开但未收到字节；检查设备输出、线缆和波特率，不能据此认定缺驱动。")
        return 3
    if parser.valid == 0:
        print("FAIL: 收到字节，但没有有效 YESENSE 帧；首 64 字节: " + prefix.hex(" "))
        return 4
    if imu_frames < 3:
        print("FAIL: 有效协议帧已收到，但包含已支持 IMU 字段的帧不足 3 个；检查输出配置或延长时间。")
        return 5
    print("PASS: 已读取并校验 IMU 数据（仅验证通信与解码，不代表精度或标定验收）。")
    return 0


def main():
    cli = argparse.ArgumentParser(description="H30 mini Linux 串口只接收测试，无需 ROS/pyserial。")
    cli.add_argument("--port", help="指定串口；默认仅自动选择唯一的 1a86:55d4 设备")
    cli.add_argument("--baud", type=int, default=460800, help="默认 460800")
    cli.add_argument("--duration", type=positive_seconds, default=10.0, help="采集秒数，默认 10")
    cli.add_argument("--csv", type=Path,
                     help="CSV 保存路径；默认 Imu_test/artifacts/imu_<UTC时间>.csv，不覆盖已有文件")
    cli.add_argument("--raw", type=Path, help="可选：保存原始二进制数据，文件必须尚不存在")
    cli.add_argument("--list", action="store_true", help="列出串口、USB ID 和驱动后退出")
    args = cli.parse_args()
    if not hasattr(termios, "B%d" % args.baud):
        cli.error("此系统的 termios 不支持指定波特率")
    try:
        devices = ports()
        if args.list:
            for path, vendor, product, driver in devices:
                print("%s USB=%s:%s driver=%s accessible=%s" %
                      (path, vendor, product, driver, os.access(path, os.R_OK | os.W_OK)))
            if not devices:
                print("未发现串口。确认 USB 连接；沙箱中请在主机终端执行。")
            return 0
        path = args.port
        if path is None:
            matches = [p for p, v, d, _ in devices if (v, d) == ("1a86", "55d4")]
            if len(matches) != 1:
                print("FAIL: 找到 %d 个 CH9102 候选设备。先用 --list 查看，再用 --port 指定。" % len(matches), file=sys.stderr)
                return 2
            path = matches[0]
        return run(args, path)
    except PermissionError as exc:
        print("FAIL: 串口或输出文件权限不足: %s\n串口授权方法见 Imu_test/README.md；这不等于缺驱动。" % exc, file=sys.stderr)
        return 2
    except (OSError, termios.error) as exc:
        print("FAIL: I/O 错误: %s" % exc, file=sys.stderr)
        return 6
    except KeyboardInterrupt:
        print("\n已停止并关闭串口。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
