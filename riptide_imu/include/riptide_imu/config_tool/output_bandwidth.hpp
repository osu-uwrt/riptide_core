#pragma once

#include <QMap>
#include <QString>
#include <QStringList>
#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace riptide_imu {

// What the sensor sends without being asked (ASCII async output, registers 6 / 7, and the three binary outputs,
// 75-77) and what that costs on the serial line (register 5). Field sizes and rates from the VN-100 manual,
// section 4 (binary output) and 5.2 (ASCII async).

constexpr double kImuRateHz = 800.0;  // binary output rates are this divided by the rate divisor
constexpr int kBinaryGroupCount = 7;  // common, time, IMU, GNSS, attitude, INS, GNSS2

// One field of a binary output group the VN-100 can produce.
struct BinaryField {
  int group;
  int bit;
  const char* name;  // as the manual names it
  const char* description;
  int bytes;
};
const std::vector<BinaryField>& binaryOutputFields();
const char* binaryGroupName(int group);
// Groups the VN-100 has fields in (the rest are GNSS / INS products).
const std::vector<int>& vn100BinaryGroups();

struct BinaryOutputConfig {
  int async_mode{0};  // 0 off, 1 serial port 1, 2 serial port 2, 3 both
  int rate_divisor{0};
  std::array<uint16_t, kBinaryGroupCount> fields{};
  bool hasOutput() const;
  double rateHz() const;
  int packetBytes() const;
  // Bits set in a group that the VN-100 has no field for.
  QStringList unknownFields() const;
};
std::optional<BinaryOutputConfig> parseBinaryOutput(const QString& semicolon_values, QString* error = nullptr);
QString formatBinaryOutput(const BinaryOutputConfig& config);

// The ASCII message for an async data output type (register 6), with an approximate length on the wire.
struct AsciiOutput {
  int id;
  const char* header;
  const char* description;
  int bytes;
};
const std::vector<AsciiOutput>& asciiOutputs();
const std::vector<int>& asciiOutputRatesHz();

struct BandwidthEstimate {
  bool known{false};      // every register it needs has a value
  QString missing;        // the ones it does not, e.g. "5, 75"
  int baud{0};
  double bits_per_second{0.0};
  double fraction() const { return baud > 0 ? bits_per_second / baud : 0.0; }
  bool exceeds() const { return known && fraction() > 1.0; }
  QStringList parts;      // "Binary output 1: 76 B at 400 Hz", in the order they add up
  QStringList notes;      // anything not counted, such as output on the other port
  double output_rate_hz{0.0};  // the fastest output on this port
};

// Load on serial port 1, the one this tool and the driver use: ASCII async output plus every binary output routed
// to port 1. Each byte is 10 bits on the line (8N1).
BandwidthEstimate estimateBandwidth(const QMap<int, QString>& registers);

}  // namespace riptide_imu
