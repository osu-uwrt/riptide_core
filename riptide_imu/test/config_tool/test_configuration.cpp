#include <gtest/gtest.h>
#include <QFile>
#include <QTemporaryDir>

#include "riptide_imu/config_tool/configuration.hpp"

TEST(ControlCenterXml, RoundTripsDualPortAndUnknownRegisters) {
  QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
  const QString first=directory.filePath("first.xml");
  QFile source(first); ASSERT_TRUE(source.open(QIODevice::WriteOnly));
  source.write("<?xml version=\"1.0\"?><VectorNav_Sensor><Model>VN-100T-CR</Model>"
               "<HardwareRevision>7</HardwareRevision><SerialNumber>42</SerialNumber>"
               "<FirmwareVersion>3.0.0.0</FirmwareVersion><Register><Id>5</Id>"
               "<Name>BaudRate</Name><Values>230400;115200</Values></Register>"
               "<Register><Id>999</Id><Name>Future</Name><Values>a;b</Values></Register>"
               "</VectorNav_Sensor>"); source.close();
  QString error; const auto parsed=riptide_imu::loadControlCenterXml(first,&error);
  ASSERT_TRUE(error.isEmpty()) << error.toStdString(); ASSERT_EQ(parsed.registers.size(),2);
  EXPECT_EQ(parsed.registers[0].values,"230400;115200");
  const QString second=directory.filePath("second.xml"); ASSERT_TRUE(riptide_imu::saveControlCenterXml(second,parsed,&error));
  const auto reparsed=riptide_imu::loadControlCenterXml(second,&error);
  ASSERT_EQ(reparsed.registers.size(),2); EXPECT_EQ(reparsed.registers[1].id,999); EXPECT_EQ(reparsed.registers[1].values,"a;b");
}

TEST(RegisterValidation, EnforcesArityAndProtocolCharacters) {
  riptide_imu::RegisterDefinition definition; definition.id=25; definition.writable=true; definition.value_count=3;
  QString error; EXPECT_TRUE(riptide_imu::validateRegisterValue(definition,"1;2;3",&error));
  EXPECT_FALSE(riptide_imu::validateRegisterValue(definition,"1;2",&error));
  EXPECT_FALSE(riptide_imu::validateRegisterValue(definition,"1;$bad;3",&error));
  definition.writable=false; EXPECT_FALSE(riptide_imu::validateRegisterValue(definition,"1;2;3",&error));
}
