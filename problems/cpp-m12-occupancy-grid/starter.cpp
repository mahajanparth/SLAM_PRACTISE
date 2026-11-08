#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <vector>

struct Vec2 { double x, y; };
struct Cell { int x, y; };

class OccupancyGrid {
 public:
  static constexpr double kMinLogOdds = -4.0;
  static constexpr double kMaxLogOdds = 4.0;

  OccupancyGrid(int width, int height, double resolution, double origin_x, double origin_y) {
    // TODO: validate, then initialise
  }
  static std::optional<OccupancyGrid> create(int width, int height, double resolution,
                                             double origin_x, double origin_y) {
    // TODO
    return std::nullopt;
  }

  int width() const { return 0; /* TODO */ }
  int height() const { return 0; /* TODO */ }
  double resolution() const { return 0.0; /* TODO */ }

  bool inBounds(int cx, int cy) const {
    // TODO
    return false;
  }
  std::optional<Cell> worldToCell(double x, double y) const {
    // TODO
    return std::nullopt;
  }
  Vec2 cellCenter(int cx, int cy) const {
    // TODO
    return {0.0, 0.0};
  }
  double logOdds(int cx, int cy) const {
    // TODO
    return 0.0;
  }
  void update(int cx, int cy, double delta) {
    // TODO
  }
  double probability(int cx, int cy) const {
    // TODO
    return 0.0;
  }
  std::size_t countOccupied(double p_thresh) const {
    // TODO
    return 0;
  }

 private:
  // TODO: your members (think about declaration order!)
};
