#include <gtest/gtest.h>

#include "riptide_imu/driver/config_gateway.hpp"

namespace riptide_imu::driver {

TEST(ConfigGateway, ParsesVersionedRequestsAndRejectsMalformedInput) {
  std::string error;
  auto request = parseConfigRequest(
      R"({"version":1,"operation":"read","payload":{"register":54}})", &error);
  ASSERT_TRUE(request);
  EXPECT_EQ(request->operation, "read");
  EXPECT_EQ(request->payload.at("register"), 54);
  EXPECT_FALSE(parseConfigRequest(R"({"version":2,"operation":"read"})", &error));
  EXPECT_FALSE(error.empty());
}

TEST(ConfigGateway, MaintenanceLeaseIsExclusiveRenewableAndExpiring) {
  const auto start = MaintenanceLease::Clock::time_point{};
  MaintenanceLease lease(std::chrono::milliseconds(100));
  const std::string token = lease.open("ui-a", start);
  ASSERT_FALSE(token.empty());
  EXPECT_TRUE(lease.valid(token, start + std::chrono::milliseconds(50)));
  EXPECT_TRUE(lease.open("ui-b", start + std::chrono::milliseconds(50)).empty());
  EXPECT_TRUE(lease.renew(token, start + std::chrono::milliseconds(80)));
  EXPECT_TRUE(lease.valid(token, start + std::chrono::milliseconds(150)));
  EXPECT_FALSE(lease.valid(token, start + std::chrono::milliseconds(181)));
  EXPECT_FALSE(lease.close("wrong"));
}

TEST(ConfigGateway, ClassifiesOnlyDriverDisruptiveRegistersAsCritical) {
  EXPECT_TRUE(isDriverCriticalRegister(5));
  EXPECT_TRUE(isDriverCriticalRegister(26));
  EXPECT_TRUE(isDriverCriticalRegister(75));
  EXPECT_FALSE(isDriverCriticalRegister(23));
  EXPECT_FALSE(isDriverCriticalRegister(25));
  EXPECT_FALSE(isDriverCriticalRegister(84));
}

TEST(ConfigGateway, ResponsesAlwaysCarryStatusAndProtocolVersion) {
  const auto response = Json::parse(configResponse(false, "busy", "calibration active"));
  EXPECT_EQ(response.at("version"), 1);
  EXPECT_FALSE(response.at("ok"));
  EXPECT_EQ(response.at("code"), "busy");
}

TEST(ConfigGateway, BaudPlanWritesInactivePortBeforeSwitchingActivePort) {
  const std::vector<uint32_t> supported{115200, 230400, 460800};
  std::string error;
  auto port_one = planBaudTransition(supported, {230400, 115200}, 1, &error);
  ASSERT_TRUE(port_one);
  EXPECT_EQ(port_one->active_baud, 230400u);
  ASSERT_TRUE(port_one->inactive_write);
  EXPECT_EQ(port_one->inactive_write->first, 2);
  EXPECT_EQ(port_one->inactive_write->second, 115200u);

  auto port_two = planBaudTransition(supported, {230400, 115200}, 2, &error);
  ASSERT_TRUE(port_two);
  EXPECT_EQ(port_two->active_baud, 115200u);
  ASSERT_TRUE(port_two->inactive_write);
  EXPECT_EQ(port_two->inactive_write->first, 1);
  EXPECT_FALSE(planBaudTransition(supported, {921600}, 1, &error));
}

TEST(ConfigGateway, PersistenceComparisonDetectsMissingAndChangedValues) {
  const Json expected{{"5", {"460800", "115200"}}, {"84", {"1", "2"}}};
  EXPECT_TRUE(persistentMismatches(expected, expected).empty());
  const Json actual{{"5", {"115200", "115200"}}};
  const Json mismatch = persistentMismatches(expected, actual);
  EXPECT_TRUE(mismatch.contains("5"));
  EXPECT_TRUE(mismatch.contains("84"));
  const Json echoed{{"5", {"460800", "115200"}}, {"84", {"1.0", "2"}}};
  EXPECT_TRUE(persistentMismatches(expected, echoed).empty());
}

TEST(ConfigGateway, ReadbackAcceptsTheSensorsSixDigitEcho) {
  // From a real calibration write: the staged bias and the sensor's float32 echo of it.
  EXPECT_TRUE(registerValuesMatch({"1", "0", "-0.00847906899", "-0.121211335", "0.000140408898"},
                                  {"1", "0", "-0.00847907", "-0.121211", "0.000140409"}));
  EXPECT_TRUE(registerFieldsMatch("-0.0003521", "-0.000352100004"));
  EXPECT_TRUE(registerFieldsMatch("0530", "530"));
  EXPECT_TRUE(registerFieldsMatch("0a30", "0A30"));
  EXPECT_TRUE(registerFieldsMatch("+00.000", "0"));
  EXPECT_FALSE(registerFieldsMatch("-0.008479", "-0.008480"));
  EXPECT_FALSE(registerFieldsMatch("460800", "460801"));
  EXPECT_FALSE(registerFieldsMatch("TALOS", "TAL0S"));
  EXPECT_FALSE(registerValuesMatch({"1", "2"}, {"1"}));
}

}  // namespace riptide_imu::driver
