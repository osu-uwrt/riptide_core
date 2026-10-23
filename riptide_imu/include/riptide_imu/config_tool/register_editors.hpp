#pragma once

#include <QMap>
#include <QWidget>

#include "riptide_imu/config_tool/output_bandwidth.hpp"

class QCheckBox;
class QComboBox;
class QLabel;

namespace riptide_imu {

// Binary output registers (75-77) as the choices they encode: which port, how often, and a checkbox per field of
// each VN-100 group. Bits it has no checkbox for are kept as they are.
class BinaryOutputEditor : public QWidget {
  Q_OBJECT
 public:
  explicit BinaryOutputEditor(QWidget* parent = nullptr);
  // `device` is what the sensor has (may be empty), `shown` the value to edit (staged, else device). `registers`
  // supplies the baud rate and the other outputs for the load figure.
  void setValues(int register_id, const QString& device, const QString& shown, const QMap<int, QString>& registers);
  void setEditable(bool editable);

 signals:
  void edited(const QString& values);

 private:
  void emitEdited();
  void refreshSummary();
  int register_id_{75};
  BinaryOutputConfig config_;
  std::optional<BinaryOutputConfig> device_;
  QMap<int, QString> registers_;
  bool updating_{false};
  QComboBox* port_{nullptr};
  QComboBox* rate_{nullptr};
  QLabel* summary_{nullptr};
  QLabel* problem_{nullptr};
  QMap<QCheckBox*, std::pair<int, int>> boxes_;  // group, bit
};

}  // namespace riptide_imu
