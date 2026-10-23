#include "riptide_imu/config_tool/register_editors.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QStyle>
#include <QVBoxLayout>

#include "riptide_imu/config_tool/dashboard_widgets.hpp"
#include "riptide_imu/config_tool/theme.hpp"

namespace riptide_imu {
namespace {

const int kDivisors[] = {1, 2, 4, 5, 8, 10, 16, 20, 25, 40, 50, 80, 100, 160, 200, 400, 800};

QString rateText(int divisor) {
  return divisor > 0 ? QString("%1 Hz  (800 ÷ %2)").arg(kImuRateHz / divisor, 0, 'g', 4).arg(divisor)
                     : QString("Off  (divisor 0)");
}

}  // namespace

BinaryOutputEditor::BinaryOutputEditor(QWidget* parent) : QWidget(parent) {
  setObjectName("binaryOutputEditor");
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 4, 0, 4);
  layout->setSpacing(10);

  auto* top = new QGridLayout;
  top->setHorizontalSpacing(12);
  top->setVerticalSpacing(6);
  auto* port_label = toneLabel("SEND ON", "muted");
  auto* rate_label = toneLabel("RATE", "muted");
  port_ = new QComboBox;
  port_->setObjectName("binaryOutputPort");
  port_->addItem("Off", 0);
  port_->addItem("Serial port 1", 1);
  port_->addItem("Serial port 2", 2);
  port_->addItem("Both ports", 3);
  rate_ = new QComboBox;
  rate_->setObjectName("binaryOutputRate");
  for (int divisor : kDivisors) rate_->addItem(rateText(divisor), divisor);
  top->addWidget(port_label, 0, 0);
  top->addWidget(port_, 1, 0);
  top->addWidget(rate_label, 0, 1);
  top->addWidget(rate_, 1, 1);
  top->setColumnStretch(2, 1);
  layout->addLayout(top);

  summary_ = toneLabel(QString(), "muted");
  summary_->setObjectName("binaryOutputSummary");
  layout->addWidget(summary_);
  problem_ = toneLabel(QString(), "error");
  problem_->hide();
  layout->addWidget(problem_);

  // Two columns: common over IMU, time over attitude.
  auto* groups = new QHBoxLayout;
  groups->setSpacing(28);
  QVBoxLayout* columns[2] = {new QVBoxLayout, new QVBoxLayout};
  for (auto* column : columns) {
    column->setSpacing(3);
    groups->addLayout(column, 1);
  }
  int index = 0;
  for (int group : vn100BinaryGroups()) {
    auto* column = columns[index++ % 2];
    if (column->count()) column->addSpacing(10);
    column->addWidget(sectionTitle(QString("%1 group").arg(binaryGroupName(group))));
    for (const auto& field : binaryOutputFields()) {
      if (field.group != group) continue;
      auto* box = new QCheckBox(QString("%1  ·  %2 B").arg(field.description).arg(field.bytes));
      // the manual's name, for looking it up
      box->setToolTip(QString("%1 (%2 group, bit %3), %4 bytes").arg(field.name, binaryGroupName(group))
                          .arg(field.bit).arg(field.bytes));
      boxes_.insert(box, {field.group, field.bit});
      column->addWidget(box);
      connect(box, &QCheckBox::toggled, this, [this, box](bool on) {
        if (updating_) return;
        const auto [g, bit] = boxes_.value(box);
        if (on) config_.fields[g] |= static_cast<uint16_t>(1u << bit);
        else config_.fields[g] &= static_cast<uint16_t>(~(1u << bit));
        emitEdited();
      });
    }
  }
  for (auto* column : columns) column->addStretch();
  layout->addLayout(groups);
  layout->addStretch();

  connect(port_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] {
    if (updating_) return;
    config_.async_mode = port_->currentData().toInt();
    emitEdited();
  });
  connect(rate_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] {
    if (updating_) return;
    config_.rate_divisor = rate_->currentData().toInt();
    emitEdited();
  });
}

void BinaryOutputEditor::setValues(int register_id, const QString& device, const QString& shown,
                                   const QMap<int, QString>& registers) {
  register_id_ = register_id;
  registers_ = registers;
  device_ = device.isEmpty() ? std::nullopt : parseBinaryOutput(device);
  QString error;
  const auto parsed = parseBinaryOutput(shown, &error);
  updating_ = true;
  config_ = parsed.value_or(BinaryOutputConfig{});
  problem_->setVisible(!shown.isEmpty() && !parsed);
  problem_->setText("Cannot read this value: " + error + ". Edit it in the raw field below.");
  port_->setCurrentIndex(port_->findData(config_.async_mode));
  int rate_index = rate_->findData(config_.rate_divisor);
  if (rate_index < 0) {
    rate_->addItem(rateText(config_.rate_divisor), config_.rate_divisor);
    rate_index = rate_->count() - 1;
  }
  rate_->setCurrentIndex(rate_index);
  for (auto box = boxes_.begin(); box != boxes_.end(); ++box) {
    const auto [group, bit] = box.value();
    const bool on = config_.fields[group] >> bit & 1;
    box.key()->setChecked(on);
    // what the sensor has differs: bold, so pending changes stand out
    const bool device_on = device_ && (device_->fields[group] >> bit & 1);
    QFont font = box.key()->font();
    font.setBold(device_ && on != device_on);
    box.key()->setFont(font);
  }
  updating_ = false;
  refreshSummary();
}

void BinaryOutputEditor::setEditable(bool editable) {
  port_->setEnabled(editable);
  rate_->setEnabled(editable);
  for (auto* box : boxes_.keys()) box->setEnabled(editable);
}

void BinaryOutputEditor::emitEdited() {
  refreshSummary();
  emit edited(formatBinaryOutput(config_));
}

void BinaryOutputEditor::refreshSummary() {
  QMap<int, QString> projected = registers_;
  projected[register_id_] = formatBinaryOutput(config_);
  const auto estimate = estimateBandwidth(projected);
  QString text;
  if (!config_.hasOutput()) {
    text = "This output is off.";
  } else {
    text = QString("%1-byte packet at %2 Hz = %3 kbit/s on the line")
               .arg(config_.packetBytes()).arg(config_.rateHz(), 0, 'g', 4)
               .arg(config_.packetBytes() * 10.0 * config_.rateHz() / 1000.0, 0, 'f', 1);
    if (!(config_.async_mode & 1)) text += " (on port 2, not counted toward port 1's load)";
  }
  if (estimate.known)
    text += QString(". All output on port 1: %1% of %2 baud.").arg(100 * estimate.fraction(), 0, 'f', 0)
                .arg(estimate.baud);
  else
    text += ". Port 1 load unknown until register " + estimate.missing + " is read.";
  const QStringList unknown = config_.unknownFields();
  if (!unknown.isEmpty()) text += " Kept as they are (not VN-100 fields): " + unknown.join(", ") + ".";
  summary_->setText(text);
  const QString tone = estimate.exceeds() ? "error" : "muted";
  if (summary_->property("tone").toString() != tone) {
    summary_->setProperty("tone", tone);
    summary_->style()->unpolish(summary_);
    summary_->style()->polish(summary_);
  }
}

}  // namespace riptide_imu
