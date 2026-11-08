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
  // TODO
  return -1.0;
}

template <class T>
std::vector<T> extract(const std::vector<Msg>& msgs) {
  // TODO
  return {};
}

void sortByStamp(std::vector<Msg>& msgs) {
  // TODO
}

Summary process(const std::vector<Msg>& msgs) {
  // TODO
  return {};
}

std::optional<OdomMsg> latestOdomBefore(const std::vector<Msg>& msgs, double t) {
  // TODO
  return std::nullopt;
}
