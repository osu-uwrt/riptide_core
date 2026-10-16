#pragma once

#include <QJsonObject>
#include <QMap>
#include <QString>
#include <QVector>

namespace riptide_imu {

struct RegisterFieldDefinition {
  QString name;
  QString units;
  QString type{"number"};
  double minimum{0.0};
  double maximum{0.0};
  bool has_range{false};
  QMap<QString, QString> enum_values;
};

struct RegisterDefinition {
  int id{-1};
  QString name;
  QString group;
  QString description;
  QString units;
  int value_count{-1};
  bool writable{false};
  bool dangerous{false};
  bool driver_critical{false};
  bool persistent{true};
  QString editor{"raw"};
  QString firmware;
  QVector<RegisterFieldDefinition> fields;
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
