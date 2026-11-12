#include <legged_hw/H30Imu.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

namespace {
void appendInt(std::vector<uint8_t>& bytes, int32_t value) {
  for (int i = 0; i < 4; ++i) bytes.push_back((uint32_t(value) >> (8 * i)) & 255);
}

std::vector<uint8_t> packet(std::vector<uint8_t> payload, uint16_t tid = 42) {
  std::vector<uint8_t> bytes{0x59, 0x53, uint8_t(tid & 255), uint8_t(tid >> 8), uint8_t(payload.size())};
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  unsigned a = 0, b = 0;
  for (size_t i = 2; i < bytes.size(); ++i) {
    a += bytes[i];
    b += a;
  }
  bytes.push_back(a & 255);
  bytes.push_back(b & 255);
  return bytes;
}

std::vector<uint8_t> payload() {
  std::vector<uint8_t> bytes{0x10, 12};
  for (int32_t v : {-1000000, 0, 9806650}) appendInt(bytes, v);
  bytes.insert(bytes.end(), {0x20, 12});
  for (int32_t v : {180000000, -90000000, 0}) appendInt(bytes, v);
  bytes.insert(bytes.end(), {0x41, 16});
  for (int32_t v : {707107, 0, 0, 707107}) appendInt(bytes, v);
  return bytes;
}
}  // namespace

TEST(H30Parser, FragmentedFramesUnitsAndQuaternionOrder) {
  legged::H30Parser parser;
  const auto frame = packet(payload());
  std::vector<legged::H30Sample> samples;
  for (uint8_t v : frame) {
    const auto parsed = parser.feed(&v, 1);
    samples.insert(samples.end(), parsed.begin(), parsed.end());
  }
  ASSERT_EQ(samples.size(), 1);
  EXPECT_EQ(samples[0].tid, 42);
  EXPECT_DOUBLE_EQ(samples[0].linearAcceleration[0], -1.);
  EXPECT_NEAR(samples[0].linearAcceleration[2], 9.80665, 1e-9);
  EXPECT_NEAR(samples[0].angularVelocity[0], std::acos(-1.), 1e-9);
  EXPECT_NEAR(samples[0].angularVelocity[1], -std::acos(-1.) / 2, 1e-9);
  EXPECT_NEAR(samples[0].orientation[2], std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(samples[0].orientation[3], std::sqrt(0.5), 1e-9);
}

TEST(H30Parser, NoiseChecksumAndBackToBackRecovery) {
  auto bad = packet(payload());
  bad.back() ^= 1;
  auto good = packet(payload(), 65535);
  std::vector<uint8_t> stream{0, 1, 0x59};
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), good.begin(), good.end());
  stream.insert(stream.end(), good.begin(), good.end());
  legged::H30Parser parser;
  auto samples = parser.feed(stream.data(), stream.size());
  ASSERT_EQ(samples.size(), 2);
  EXPECT_EQ(samples[0].tid, 65535);
  EXPECT_EQ(parser.checksumErrors(), 1);
}

TEST(H30Parser, RejectsIncompleteFieldsAndMalformedTlv) {
  legged::H30Parser parser;
  for (const auto& bytes : {packet({0x10, 12, 0}), packet({0x10, 1, 0}), packet({0x01, 2, 0, 0})}) {
    EXPECT_TRUE(parser.feed(bytes.data(), bytes.size()).empty());
  }
  const auto good = packet(payload());
  EXPECT_EQ(parser.feed(good.data(), good.size()).size(), 1);
  EXPECT_EQ(parser.payloadErrors(), 3);
}

TEST(H30Parser, RejectsZeroQuaternionAndAcceptsUnknownTlv) {
  auto data = payload();
  std::fill(data.end() - 16, data.end(), 0);
  auto bytes = packet(data);
  legged::H30Parser parser;
  EXPECT_TRUE(parser.feed(bytes.data(), bytes.size()).empty());
  data = payload();
  data.insert(data.end(), {0xf0, 3, 1, 2, 3});
  bytes = packet(data);
  EXPECT_EQ(parser.feed(bytes.data(), bytes.size()).size(), 1);
}

TEST(H30Serial, NonblockingReceiveOnlyAndRestoreSettings) {
  int master, slave;
  char path[128];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  termios before{}, after{};
  ASSERT_EQ(tcgetattr(slave, &before), 0);
  {
    legged::H30Serial serial(path, 460800);
    ASSERT_EQ(tcgetattr(slave, &after), 0);
    EXPECT_EQ(cfgetispeed(&after), B460800);
    EXPECT_FALSE(after.c_lflag & (ECHO | ICANON));
    EXPECT_TRUE(serial.poll().empty());
    const auto frame = packet(payload());
    ASSERT_EQ(write(master, frame.data(), frame.size()), ssize_t(frame.size()));
    std::vector<legged::H30Sample> samples;
    for (int i = 0; i < 100 && samples.empty(); ++i) {
      samples = serial.poll();
      if (samples.empty()) usleep(1000);
    }
    EXPECT_EQ(samples.size(), 1);
    fcntl(master, F_SETFL, O_NONBLOCK);
    uint8_t byte;
    EXPECT_EQ(read(master, &byte, 1), -1);  // No echo or transmitted data.
  }
  ASSERT_EQ(tcgetattr(slave, &after), 0);
  EXPECT_EQ(before.c_cflag, after.c_cflag);
  EXPECT_EQ(before.c_lflag, after.c_lflag);
  EXPECT_EQ(cfgetispeed(&before), cfgetispeed(&after));
  close(master);
  close(slave);
}
