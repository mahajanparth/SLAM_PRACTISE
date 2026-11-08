#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <utility>
#include <vector>

struct Point3 { double x, y, z; };

std::vector<std::pair<std::size_t, std::size_t>> matchTimestamps(const std::vector<double>& lidar,
                                                                 const std::vector<double>& cam, double max_dt) {
  // TODO: std::lower_bound per lidar stamp
  return {};
}

std::size_t partitionGround(std::vector<Point3>& cloud, double z_thresh) {
  // TODO: std::stable_partition
  return 0;
}
