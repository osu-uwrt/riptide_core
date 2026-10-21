#include "riptide_imu/config_tool/configuration.hpp"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>

#include "riptide_imu/driver/config_gateway.hpp"

namespace riptide_imu {

QVector<RegisterDefinition> loadRegisterDefinitions(const QString& path,
                                                    QString* error) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = file.errorString();
    return {};
  }
  QJsonParseError parse_error;
  const auto document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    if (error) *error = parse_error.errorString();
    return {};
  }
  QVector<RegisterDefinition> result;
  for (const auto value : document.object().value("registers").toArray()) {
    const auto object = value.toObject();
    RegisterDefinition definition;
    definition.id = object.value("id").toInt(-1);
    definition.name = object.value("name").toString();
    definition.group = object.value("group").toString();
    definition.description = object.value("description").toString();
    definition.units = object.value("units").toString();
    definition.value_count = object.value("value_count").toInt(-1);
    definition.writable = object.value("writable").toBool(false);
    definition.dangerous = object.value("dangerous").toBool(false);
    definition.driver_critical = object.value("driver_critical").toBool(false);
    definition.persistent = object.value("persistent").toBool(true);
    definition.editor = object.value("editor").toString("raw");
    definition.firmware = object.value("firmware").toString();
    for (const auto field_value : object.value("fields").toArray()) {
      const auto field_object = field_value.toObject();
      RegisterFieldDefinition field;
      field.name = field_object.value("name").toString();
      field.units = field_object.value("units").toString();
      field.type = field_object.value("type").toString("number");
      if (field_object.contains("minimum") && field_object.contains("maximum")) {
        field.has_range = true;
        field.minimum = field_object.value("minimum").toDouble();
        field.maximum = field_object.value("maximum").toDouble();
      }
      const auto enum_object = field_object.value("values").toObject();
      for (auto entry = enum_object.constBegin(); entry != enum_object.constEnd(); ++entry)
        field.enum_values.insert(entry.key(), entry.value().toString());
      definition.fields.push_back(field);
    }
    if (definition.id >= 0 && !definition.name.isEmpty()) result.push_back(definition);
  }
  if (result.isEmpty() && error) *error = "Register schema contains no registers";
  return result;
}

SensorConfiguration loadControlCenterXml(const QString& path, QString* error) {
  SensorConfiguration result;
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = file.errorString();
    return result;
  }
  QXmlStreamReader xml(&file);
  RegisterValue current;
  while (!xml.atEnd()) {
    xml.readNext();
    if (!xml.isStartElement()) continue;
    const QString name = xml.name().toString();
    if (name == "Model") result.model = xml.readElementText();
    else if (name == "HardwareRevision") result.hardware_revision = xml.readElementText();
    else if (name == "SerialNumber") result.serial_number = xml.readElementText();
    else if (name == "FirmwareVersion") result.firmware_version = xml.readElementText();
    else if (name == "Register") {
      current = {};
      while (!(xml.isEndElement() && xml.name() == "Register") && !xml.atEnd()) {
        xml.readNext();
        if (!xml.isStartElement()) continue;
        if (xml.name() == "Id") current.id = xml.readElementText().toInt();
        else if (xml.name() == "Name") current.name = xml.readElementText();
        else if (xml.name() == "Values") current.values = xml.readElementText();
        else xml.skipCurrentElement();
      }
      if (current.id >= 0) result.registers.push_back(current);
    }
  }
  if (xml.hasError() && error) *error = xml.errorString();
  return result;
}

bool saveControlCenterXml(const QString& path,
                          const SensorConfiguration& configuration,
                          QString* error) {
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    if (error) *error = file.errorString();
    return false;
  }
  QXmlStreamWriter xml(&file);
  xml.setAutoFormatting(true);
  xml.writeStartDocument();
  xml.writeStartElement("VectorNav_Sensor");
  xml.writeAttribute("xmlns:xsd", "http://www.w3.org/2001/XMLSchema");
  xml.writeAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
  xml.writeTextElement("Model", configuration.model);
  xml.writeTextElement("HardwareRevision", configuration.hardware_revision);
  xml.writeTextElement("SerialNumber", configuration.serial_number);
  xml.writeTextElement("FirmwareVersion", configuration.firmware_version);
  for (const auto& reg : configuration.registers) {
    xml.writeStartElement("Register");
    xml.writeTextElement("Id", QString::number(reg.id));
    xml.writeTextElement("Name", reg.name);
    xml.writeTextElement("Values", reg.values);
    xml.writeEndElement();
  }
  xml.writeEndElement();
  xml.writeEndDocument();
  return !file.error();
}

bool validateRegisterValue(const RegisterDefinition& definition,
                           const QString& values, QString* error) {
  if (!definition.writable) {
    if (error) *error = "Register is read-only";
    return false;
  }
  const auto fields = values.split(';', Qt::KeepEmptyParts);
  if (definition.value_count >= 0 && fields.size() != definition.value_count) {
    if (error) *error = QString("Expected %1 values; received %2")
                            .arg(definition.value_count).arg(fields.size());
    return false;
  }
  for (const auto& field : fields) {
    if (field.trimmed().isEmpty() || field.contains(',') || field.contains('*') ||
        field.contains('$') || field.contains('\r') || field.contains('\n')) {
      if (error) *error = "Values contain an empty or protocol-reserved field";
      return false;
    }
  }
  return true;
}

bool registerValuesMatch(const QString& written, const QString& read) {
  std::vector<std::string> a, b;
  for (const auto& field : written.split(';')) a.push_back(field.toStdString());
  for (const auto& field : read.split(';')) b.push_back(field.toStdString());
  return driver::registerValuesMatch(a, b);
}

}  // namespace riptide_imu
