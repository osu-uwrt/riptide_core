#pragma once

#include <QJsonObject>
#include <QString>
#include <QVector>

namespace riptide_imu {

struct RegisterDefinition {
  int id{-1};
  QString name;
  QString group;
  QString description;
  QString units;
  int value_count{-1};
  bool writable{false};
  bool dangerous{false};
};

struct RegisterValue {
  int id{-1};
  QString name;
  QString values;
  bool supported{false};
};

struct SensorConfiguration {
  QString model;
  QString hardware_revision;
  QString serial_number;
  QString firmware_version;
  QVector<RegisterValue> registers;
};

QVector<RegisterDefinition> loadRegisterDefinitions(const QString& path,
                                                    QString* error);
SensorConfiguration loadControlCenterXml(const QString& path, QString* error);
bool saveControlCenterXml(const QString& path,
                          const SensorConfiguration& configuration,
                          QString* error);
bool validateRegisterValue(const RegisterDefinition& definition,
                           const QString& values, QString* error);

}  // namespace riptide_imu
