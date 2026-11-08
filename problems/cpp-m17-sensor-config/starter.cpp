#include <charconv>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

struct SensorConfig {
  std::string name;
  std::string frame;
  double rate_hz;
  int id;
};

std::vector<std::string_view> split(std::string_view s, char delim) {
  // TODO
  return {};
}

std::string_view trim(std::string_view s) {
  // TODO
  return {};
}

std::optional<double> parseDouble(std::string_view s) {
  // TODO
  return std::nullopt;
}

std::optional<int> parseInt(std::string_view s) {
  // TODO
  return std::nullopt;
}

std::optional<SensorConfig> parseSensor(std::string_view line) {
  // TODO
  return std::nullopt;
}

std::vector<std::string> fastSensors(const std::map<std::string, SensorConfig>& rig, double min_rate) {
  // TODO: use structured bindings
  return {};
}
