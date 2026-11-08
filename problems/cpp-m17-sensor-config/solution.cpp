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
  std::vector<std::string_view> out;
  while (true) {
    const std::size_t pos = s.find(delim);
    if (pos == std::string_view::npos) { out.push_back(s); break; }  // last token (maybe empty)
    out.push_back(s.substr(0, pos));  // O(1): views into the caller's buffer
    s.remove_prefix(pos + 1);
  }
  return out;
}

std::string_view trim(std::string_view s) {
  const std::size_t b = s.find_first_not_of(" \t");
  if (b == std::string_view::npos) return {};
  const std::size_t e = s.find_last_not_of(" \t");
  return s.substr(b, e - b + 1);
}

std::optional<double> parseDouble(std::string_view s) {
  s = trim(s);
  if (s.empty()) return std::nullopt;
  double v = 0.0;
  // from_chars: explicit [first,last) range (views aren't NUL-terminated), no locale, no throw.
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (ec != std::errc() || ptr != s.data() + s.size()) return std::nullopt;  // whole token consumed
  return v;
}

std::optional<int> parseInt(std::string_view s) {
  s = trim(s);
  if (s.empty()) return std::nullopt;
  int v = 0;
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (ec != std::errc() || ptr != s.data() + s.size()) return std::nullopt;
  return v;
}

std::optional<SensorConfig> parseSensor(std::string_view line) {
  std::optional<std::string_view> name, frame, rate_s, id_s;
  for (std::string_view seg : split(line, ';')) {
    seg = trim(seg);
    if (seg.empty()) continue;
    const std::size_t eq = seg.find('=');
    if (eq == std::string_view::npos) return std::nullopt;
    const std::string_view key = trim(seg.substr(0, eq));
    const std::string_view val = trim(seg.substr(eq + 1));
    std::optional<std::string_view>* slot = nullptr;
    if (key == "name") slot = &name;
    else if (key == "frame") slot = &frame;
    else if (key == "rate") slot = &rate_s;
    else if (key == "id") slot = &id_s;
    else return std::nullopt;            // unknown key
    if (slot->has_value()) return std::nullopt;  // duplicate key
    *slot = val;
  }
  if (!name || !frame || !rate_s || !id_s) return std::nullopt;
  if (name->empty() || frame->empty()) return std::nullopt;
  const auto rate = parseDouble(*rate_s);
  if (!rate || !std::isfinite(*rate) || *rate <= 0.0) return std::nullopt;
  const auto id = parseInt(*id_s);
  if (!id || *id < 0) return std::nullopt;
  // Only here do we allocate: the result must own its strings (the views die with `line`'s buffer).
  return SensorConfig{std::string(*name), std::string(*frame), *rate, *id};
}

std::vector<std::string> fastSensors(const std::map<std::string, SensorConfig>& rig, double min_rate) {
  std::vector<std::string> out;
  for (const auto& [key, cfg] : rig)
    if (cfg.rate_hz >= min_rate) out.push_back(key);
  return out;
}
