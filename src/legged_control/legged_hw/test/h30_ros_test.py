#!/usr/bin/env python3
"""PTY -> actual C++ H30 node -> ROS Imu, without a physical device."""
import math
import os
import pty
import struct
import subprocess
import tempfile
import termios
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import Imu


def frame():
    payload = (b"\x10\x0c" + struct.pack("<3i", 1000000, 0, 9806650) +
               b"\x20\x0c" + struct.pack("<3i", 180000000, 0, 0) +
               b"\x41\x10" + struct.pack("<4i", 707107, 0, 0, 707107))
    body = struct.pack("<HB", 123, len(payload)) + payload
    a = sum(body) & 255
    b = sum((len(body) - i) * v for i, v in enumerate(body)) & 255
    return b"\x59\x53" + body + bytes((a, b))


class H30RosTest(unittest.TestCase):
    def test_serial_ros_transform_and_timeout(self):
        master, slave = pty.openpty()
        before = termios.tcgetattr(slave)
        messages = []
        sub = rospy.Subscriber("/h30_test/imu", Imu, messages.append, queue_size=100)
        rospy.set_param("/h30_test_driver/sensor_to_base_rpy", [0., 0., math.pi / 2])
        with tempfile.TemporaryFile(mode="w+") as log:
            proc = subprocess.Popen([
                "rosrun", "legged_hw", "h30_imu_node", "__name:=h30_test_driver",
                "_port:=" + os.ttyname(slave), "_topic:=/h30_test/imu",
                "_startup_timeout:=5.0", "_data_timeout:=0.2"], stdout=log, stderr=log)
            try:
                deadline = time.monotonic() + 8
                # Wait for the reader to configure raw serial before transmitting.
                while termios.tcgetattr(slave)[3] & termios.ICANON:
                    self.assertIsNone(proc.poll(), "H30 node exited during startup")
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(0.01)
                # Keep output alive while TCPROS subscribers negotiate connections.
                end = time.monotonic() + 1.5
                while time.monotonic() < end:
                    os.write(master, frame())
                    time.sleep(0.005)
                self.assertGreater(len(messages), 10)
                msg = messages[-1]
                self.assertEqual(msg.header.frame_id, "base_imu")
                self.assertFalse(msg.header.stamp.is_zero())
                self.assertAlmostEqual(msg.angular_velocity.x, 0., places=6)
                self.assertAlmostEqual(msg.angular_velocity.y, math.pi, places=6)
                self.assertAlmostEqual(msg.linear_acceleration.x, 0., places=6)
                self.assertAlmostEqual(msg.linear_acceleration.y, 1., places=6)
                self.assertAlmostEqual(msg.linear_acceleration.z, 9.80665, places=6)
                self.assertAlmostEqual(abs(msg.orientation.w), 1., places=6)
                self.assertAlmostEqual(msg.orientation.z, 0., places=6)
                # Stop sending: the driver must exit with failure, never publish stale data forever.
                self.assertEqual(proc.wait(timeout=3), 1)
                self.assertEqual(termios.tcgetattr(slave), before)
                log.seek(0)
                self.assertIn("H30 timed out", log.read())
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=3)
                sub.unregister()
                os.close(master)
                os.close(slave)


if __name__ == "__main__":
    rospy.init_node("h30_ros_test", anonymous=True)
    rostest.rosrun("legged_hw", "h30_ros_serial", H30RosTest)
