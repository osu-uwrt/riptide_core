#include "riptide_imu/driver/config_gateway.hpp"

#include <algorithm>
#include <array>
#include <iomanip>
#include <random>
#include <sstream>

namespace riptide_imu::driver {

std::optional<ConfigRequest> parseConfigRequest(const std::string& text,
                                                std::string* error) {
  try {
    const Json document = Json::parse(text);
    if (!document.is_object()) throw std::runtime_error("request must be an object");
    ConfigRequest request;
    request.version = document.value("version", 0);
    request.operation = document.value("operation", "");
    request.session = document.value("session", "");
    request.operation_id = document.value("operation_id", "");
    request.payload = document.value("payload", Json::object());
    if (request.version != 1) throw std::runtime_error("unsupported protocol version");
    if (request.operation.empty()) throw std::runtime_error("operation is required");
    return request;
  } catch (const std::exception& exception) {
    if (error) *error = exception.what();
    return std::nullopt;
  }
}

std::string configResponse(bool ok, const std::string& code,
                           const std::string& message, const Json& data) {
  return Json{{"version", 1}, {"ok", ok}, {"code", code},
              {"message", message}, {"data", data}}.dump();
}

bool isDriverCriticalRegister(int register_id) {
  static constexpr std::array<int, 10> critical{{5, 6, 7, 26, 30, 32, 75, 76, 77, 82}};
  return std::find(critical.begin(), critical.end(), register_id) != critical.end();
}

std::optional<BaudTransitionPlan> planBaudTransition(
    const std::vector<uint32_t>& supported, const std::vector<uint32_t>& requested,
    int active_port, std::string* error) {
  if (active_port != 1 && active_port != 2) {
    if (error) *error = "active sensor port must be 1 or 2";
    return std::nullopt;
  }
  if (requested.empty() || requested.size() > 2) {
    if (error) *error = "expected one or two baud values";
    return std::nullopt;
  }
  for (const auto baud : requested) {
    if (std::find(supported.begin(), supported.end(), baud) == supported.end()) {
      if (error) *error = "unsupported baud";
      return std::nullopt;
    }
  }
  BaudTransitionPlan plan;
  plan.active_port = active_port;
  plan.active_baud = requested.at(requested.size() == 1 ? 0 : active_port - 1);
  if (requested.size() == 2) {
    const int inactive_port = active_port == 1 ? 2 : 1;
    plan.inactive_write = std::make_pair(inactive_port, requested.at(inactive_port - 1));
  }
  return plan;
}

Json persistentMismatches(const Json& expected, const Json& actual) {
  Json mismatches = Json::object();
  for (const auto& entry : expected.items()) {
    const auto actual_entry = actual.find(entry.key());
    if (actual_entry == actual.end() || *actual_entry != entry.value()) {
      mismatches[entry.key()] = {{"expected", entry.value()},
                                 {"actual", actual_entry == actual.end() ? Json() : *actual_entry}};
    }
  }
  return mismatches;
}

MaintenanceLease::MaintenanceLease(std::chrono::milliseconds duration)
    : duration_(duration) {}

std::string MaintenanceLease::open(const std::string& client_id, Clock::time_point now) {
  if (active(now) && client_id != client_id_) return {};
  client_id_ = client_id;
  std::random_device random;
  std::ostringstream token;
  token << std::hex << random() << random() << ++sequence_;
  token_ = token.str();
  expires_at_ = now + duration_;
  return token_;
}

bool MaintenanceLease::renew(const std::string& token, Clock::time_point now) {
  if (!valid(token, now)) return false;
  expires_at_ = now + duration_;
  return true;
}

bool MaintenanceLease::close(const std::string& token) {
  if (token.empty() || token != token_) return false;
  token_.clear();
  client_id_.clear();
  expires_at_ = {};
  return true;
}

bool MaintenanceLease::valid(const std::string& token, Clock::time_point now) const {
  return !token.empty() && token == token_ && now < expires_at_;
}

bool MaintenanceLease::active(Clock::time_point now) const {
  return !token_.empty() && now < expires_at_;
}

std::chrono::milliseconds MaintenanceLease::remaining(Clock::time_point now) const {
  if (!active(now)) return std::chrono::milliseconds(0);
  return std::chrono::duration_cast<std::chrono::milliseconds>(expires_at_ - now);
}

}  // namespace riptide_imu::driver
