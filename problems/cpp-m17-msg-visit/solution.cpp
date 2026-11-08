#include <algorithm>
#include <cmath>
#include <optional>
#include <type_traits>
#include <variant>
#include <vector>

struct LidarMsg { double stamp; std::vector<float> ranges; };
struct ImuMsg   { double stamp; double gyro_z; double acc_x; };
struct OdomMsg  { double stamp; double x, y, yaw; };
using Msg = std::variant<LidarMsg, ImuMsg, OdomMsg>;

struct Summary {
  int lidar = 0, imu = 0, odom = 0;
  std::size_t valid_ranges = 0;
  double yaw_from_gyro = 0.0;
  std::optional<OdomMsg> last_odom;
};

double stampOf(const Msg& m) {
  // Every alternative has `.stamp`, so one generic lambda covers all of them.
  return std::visit([](const auto& x) { return x.stamp; }, m);
}

template <class T>
std::vector<T> extract(const std::vector<Msg>& msgs) {
  std::vector<T> out;
  for (const Msg& m : msgs)
    if (const T* p = std::get_if<T>(&m)) out.push_back(*p);  // non-throwing type check
  return out;
}

void sortByStamp(std::vector<Msg>& msgs) {
  std::stable_sort(msgs.begin(), msgs.end(),
                   [](const Msg& a, const Msg& b) { return stampOf(a) < stampOf(b); });
}

Summary process(const std::vector<Msg>& msgs) {
  Summary s;
  std::optional<ImuMsg> prev_imu;
  for (const Msg& m : msgs) {
    std::visit(
        [&](const auto& x) {
          using T = std::decay_t<decltype(x)>;
          if constexpr (std::is_same_v<T, LidarMsg>) {
            ++s.lidar;
            for (float r : x.ranges)
              if (std::isfinite(r) && r > 0.0f) ++s.valid_ranges;
          } else if constexpr (std::is_same_v<T, ImuMsg>) {
            ++s.imu;
            if (prev_imu) s.yaw_from_gyro += prev_imu->gyro_z * (x.stamp - prev_imu->stamp);
            prev_imu = x;
          } else {
            // If a new alternative is added, this static_assert forces us to handle it.
            static_assert(std::is_same_v<T, OdomMsg>, "unhandled message type");
            ++s.odom;
            if (!s.last_odom || x.stamp >= s.last_odom->stamp) s.last_odom = x;
          }
        },
        m);
  }
  return s;
}

std::optional<OdomMsg> latestOdomBefore(const std::vector<Msg>& msgs, double t) {
  std::optional<OdomMsg> best;
  for (const Msg& m : msgs) {
    const auto* o = std::get_if<OdomMsg>(&m);
    if (o && o->stamp <= t && (!best || o->stamp > best->stamp)) best = *o;  // strict > keeps the first tie
  }
  return best;
}
