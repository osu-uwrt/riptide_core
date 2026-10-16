#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace riptide_imu::driver {

using Json = nlohmann::json;

struct ConfigRequest {
  int version{0};
  std::string operation;
  std::string session;
  std::string operation_id;
  Json payload{Json::object()};
};

std::optional<ConfigRequest> parseConfigRequest(const std::string& text,
                                                std::string* error);
std::string configResponse(bool ok, const std::string& code,
                           const std::string& message,
                           const Json& data = Json::object());
bool isDriverCriticalRegister(int register_id);

struct BaudTransitionPlan {
  uint32_t active_baud{0};
  int active_port{0};
  std::optional<std::pair<int, uint32_t>> inactive_write;
};

std::optional<BaudTransitionPlan> planBaudTransition(
    const std::vector<uint32_t>& supported, const std::vector<uint32_t>& requested,
    int active_port, std::string* error);
Json persistentMismatches(const Json& expected, const Json& actual);

class MaintenanceLease {
 public:
  using Clock = std::chrono::steady_clock;

  explicit MaintenanceLease(std::chrono::milliseconds duration =
                                std::chrono::seconds(30));
  std::string open(const std::string& client_id, Clock::time_point now = Clock::now());
  bool renew(const std::string& token, Clock::time_point now = Clock::now());
  bool close(const std::string& token);
  bool valid(const std::string& token, Clock::time_point now = Clock::now()) const;
  bool active(Clock::time_point now = Clock::now()) const;
  std::chrono::milliseconds remaining(Clock::time_point now = Clock::now()) const;

 private:
  std::chrono::milliseconds duration_;
  std::string client_id_;
  std::string token_;
  Clock::time_point expires_at_{};
  uint64_t sequence_{0};
};

}  // namespace riptide_imu::driver
