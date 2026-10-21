#include "riptide_imu/config_tool/output_bandwidth.hpp"

#include <algorithm>
#include <cmath>

namespace riptide_imu {

const std::vector<BinaryField>& binaryOutputFields() {
  static const std::vector<BinaryField> fields{
      {0, 0, "TimeStartup", "Time since startup", 8},
      {0, 2, "TimeSyncIn", "Time since SyncIn", 8},
      {0, 3, "YawPitchRoll", "Yaw, pitch, roll", 12},
      {0, 4, "Quaternion", "Quaternion", 16},
      {0, 5, "AngularRate", "Angular rate", 12},
      {0, 8, "Accel", "Acceleration", 12},
      {0, 9, "Imu", "Uncompensated accel + gyro", 24},
      {0, 10, "MagPres", "Magnetic, temperature, pressure", 20},
      {0, 11, "DeltaTheta", "Delta theta + delta velocity", 28},
      {0, 13, "SyncInCnt", "SyncIn count", 4},
      {1, 0, "TimeStartup", "Time since startup", 8},
      {1, 4, "TimeSyncIn", "Time since SyncIn", 8},
      {1, 7, "SyncInCnt", "SyncIn count", 4},
      {1, 8, "SyncOutCnt", "SyncOut count", 4},
      {2, 1, "UncompMag", "Uncompensated magnetic", 12},
      {2, 2, "UncompAccel", "Uncompensated accel", 12},
      {2, 3, "UncompGyro", "Uncompensated gyro", 12},
      {2, 4, "Temp", "Temperature", 4},
      {2, 5, "Pres", "Pressure", 4},
      {2, 6, "DeltaTheta", "Delta theta", 16},
      {2, 7, "DeltaVel", "Delta velocity", 12},
      {2, 8, "Mag", "Magnetic", 12},
      {2, 9, "Accel", "Acceleration", 12},
      {2, 10, "AngularRate", "Angular rate", 12},
      {4, 0, "VpeStatus", "Filter status", 2},
      {4, 1, "YawPitchRoll", "Yaw, pitch, roll", 12},
      {4, 2, "Quaternion", "Quaternion", 16},
      {4, 3, "DCM", "Rotation matrix (DCM)", 36},
      {4, 4, "MagNed", "Magnetic in NED", 12},
      {4, 5, "AccelNed", "Acceleration in NED", 12},
      {4, 6, "LinearAccelBody", "Linear accel, body", 12},
      {4, 7, "LinearAccelNed", "Linear accel, NED", 12},
      {4, 8, "YprU", "Yaw, pitch, roll uncertainty", 12},
      {4, 12, "Heave", "Heave, heave rate, delayed heave", 12}};
  return fields;
}

const char* binaryGroupName(int group) {
  static const char* names[kBinaryGroupCount] = {"Common", "Time", "IMU", "GNSS", "Attitude", "INS", "GNSS2"};
  return group >= 0 && group < kBinaryGroupCount ? names[group] : "?";
}

const std::vector<int>& vn100BinaryGroups() {
  static const std::vector<int> groups{0, 1, 2, 4};
  return groups;
}

bool BinaryOutputConfig::hasOutput() const {
  return async_mode != 0 && rate_divisor > 0 && std::any_of(fields.begin(), fields.end(), [](uint16_t f) { return f; });
}

double BinaryOutputConfig::rateHz() const { return hasOutput() ? kImuRateHz / rate_divisor : 0.0; }

int BinaryOutputConfig::packetBytes() const {
  // sync byte, group byte, a 16-bit field word per active group, the payload, then a 16-bit CRC
  int bytes = 2 + 2;
  for (int group = 0; group < kBinaryGroupCount; ++group) {
    if (!fields[group]) continue;
    bytes += 2;
    for (const auto& field : binaryOutputFields())
      if (field.group == group && (fields[group] >> field.bit & 1)) bytes += field.bytes;
  }
  return bytes;
}

QStringList BinaryOutputConfig::unknownFields() const {
  QStringList unknown;
  for (int group = 0; group < kBinaryGroupCount; ++group) {
    for (int bit = 0; bit < 16; ++bit) {
      if (!(fields[group] >> bit & 1)) continue;
      const bool known = std::any_of(binaryOutputFields().begin(), binaryOutputFields().end(),
                                     [&](const BinaryField& f) { return f.group == group && f.bit == bit; });
      if (!known) unknown << QString("%1 bit %2").arg(binaryGroupName(group)).arg(bit);
    }
  }
  return unknown;
}

std::optional<BinaryOutputConfig> parseBinaryOutput(const QString& semicolon_values, QString* error) {
  const QStringList values = semicolon_values.split(';');
  auto fail = [&](const QString& message) -> std::optional<BinaryOutputConfig> {
    if (error) *error = message;
    return std::nullopt;
  };
  if (values.size() < 3) return fail("Expected async mode, rate divisor and output groups");
  BinaryOutputConfig config;
  bool ok = false;
  config.async_mode = values[0].trimmed().toInt(&ok);
  if (!ok || config.async_mode < 0 || config.async_mode > 3) return fail("Async mode must be 0-3");
  config.rate_divisor = values[1].trimmed().toInt(&ok);
  if (!ok || config.rate_divisor < 0) return fail("Rate divisor must be a whole number");
  const uint groups = values[2].trimmed().toUInt(&ok, 16);
  if (!ok || groups > 0x7f) return fail("Output groups must be a hex byte");
  int next = 3;
  for (int group = 0; group < kBinaryGroupCount; ++group) {
    if (!(groups >> group & 1)) continue;
    if (next >= values.size()) return fail(QString("Missing the %1 group's fields").arg(binaryGroupName(group)));
    const uint word = values[next++].trimmed().toUInt(&ok, 16);
    if (!ok || word > 0xffff) return fail(QString("%1 group fields must be a hex word").arg(binaryGroupName(group)));
    config.fields[group] = static_cast<uint16_t>(word);
  }
  if (next != values.size()) return fail("More group fields than selected groups");
  return config;
}

QString formatBinaryOutput(const BinaryOutputConfig& config) {
  uint groups = 0;
  QStringList words;
  for (int group = 0; group < kBinaryGroupCount; ++group) {
    if (!config.fields[group]) continue;
    groups |= 1u << group;
    words << QString("%1").arg(config.fields[group], 4, 16, QChar('0')).toUpper();
  }
  QStringList values{QString::number(config.async_mode), QString::number(config.rate_divisor),
                     QString("%1").arg(groups, 2, 16, QChar('0')).toUpper()};
  return (values + words).join(';');
}

const std::vector<AsciiOutput>& asciiOutputs() {
  // Lengths include the header, commas, checksum and CR LF, at the sensor's fixed field widths.
  static const std::vector<AsciiOutput> outputs{
      {0, "Off", "No ASCII output", 0},
      {1, "VNYPR", "Yaw, pitch, roll", 38},
      {2, "VNQTN", "Quaternion", 51},
      {8, "VNQMR", "Quaternion, magnetic, acceleration, angular rate", 135},
      {10, "VNMAG", "Magnetic", 38},
      {11, "VNACC", "Acceleration", 35},
      {12, "VNGYR", "Angular rate", 44},
      {13, "VNMAR", "Magnetic, acceleration, angular rate", 95},
      {14, "VNYMR", "Yaw, pitch, roll, magnetic, acceleration, angular rate", 122},
      {16, "VNYBA", "Yaw, pitch, roll, body true acceleration, angular rate", 95},
      {17, "VNYIA", "Yaw, pitch, roll, inertial true acceleration, angular rate", 95},
      {19, "VNIMU", "Calibrated IMU measurements", 110},
      {30, "VNDTV", "Delta theta and delta velocity", 81}};
  return outputs;
}

const std::vector<int>& asciiOutputRatesHz() {
  static const std::vector<int> rates{1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200};
  return rates;
}

BandwidthEstimate estimateBandwidth(const QMap<int, QString>& registers) {
  BandwidthEstimate estimate;
  QStringList missing;
  for (int id : {5, 6, 7, 75, 76, 77})
    if (!registers.contains(id) || registers.value(id).trimmed().isEmpty()) missing << QString::number(id);
  estimate.missing = missing.join(", ");
  estimate.baud = registers.value(5).split(';').value(0).trimmed().toInt();

  const int ador = registers.value(6).split(';').value(0).trimmed().toInt();
  const double adof = registers.value(7).split(';').value(0).trimmed().toDouble();
  if (ador != 0 && adof > 0) {
    const auto output = std::find_if(asciiOutputs().begin(), asciiOutputs().end(),
                                     [&](const AsciiOutput& o) { return o.id == ador; });
    // An unlisted type is costed as the longest message, so the estimate never reads low.
    const int bytes = output != asciiOutputs().end() ? output->bytes : 135;
    estimate.bits_per_second += bytes * 10.0 * adof;
    estimate.output_rate_hz = std::max(estimate.output_rate_hz, adof);
    estimate.parts << QString("ASCII %1: ≈%2 B at %3 Hz")
                          .arg(output != asciiOutputs().end() ? output->header : "type " + QString::number(ador))
                          .arg(bytes).arg(adof, 0, 'g', 4);
  }
  for (int index = 0; index < 3; ++index) {
    const int id = 75 + index;
    if (!registers.contains(id)) continue;
    QString error;
    const auto config = parseBinaryOutput(registers.value(id), &error);
    if (!config) {
      estimate.notes << QString("Binary output %1 unreadable: %2").arg(index + 1).arg(error);
      continue;
    }
    if (!config->hasOutput()) continue;
    if (!(config->async_mode & 1)) {
      estimate.notes << QString("Binary output %1 goes to serial port 2 only, not counted").arg(index + 1);
      continue;
    }
    const int bytes = config->packetBytes();
    estimate.bits_per_second += bytes * 10.0 * config->rateHz();
    estimate.output_rate_hz = std::max(estimate.output_rate_hz, config->rateHz());
    estimate.parts << QString("Binary output %1: %2 B at %3 Hz").arg(index + 1).arg(bytes)
                          .arg(config->rateHz(), 0, 'g', 4);
    if (!config->unknownFields().isEmpty())
      estimate.notes << QString("Binary output %1 has fields this tool cannot size: %2")
                            .arg(index + 1).arg(config->unknownFields().join(", "));
  }
  estimate.known = missing.isEmpty() && estimate.baud > 0;
  return estimate;
}

}  // namespace riptide_imu
