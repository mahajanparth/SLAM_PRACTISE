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

  OccupancyGrid(int width, int height, double resolution, double origin_x, double origin_y)
      : w_(checked(width > 0, width, "width")),
        h_(checked(height > 0, height, "height")),
        res_(checked(resolution > 0 && std::isfinite(resolution), resolution, "resolution")),
        ox_(origin_x),
        oy_(origin_y),
        cells_(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_), 0.0) {}
  // ^ members are initialised in DECLARATION order (w_, h_, res_, ox_, oy_, cells_), so
  //   cells_ may safely use w_ and h_.

  static std::optional<OccupancyGrid> create(int width, int height, double resolution,
                                             double origin_x, double origin_y) {
    if (width <= 0 || height <= 0 || !(resolution > 0) || !std::isfinite(resolution))
      return std::nullopt;
    return OccupancyGrid(width, height, resolution, origin_x, origin_y);
  }

  int width() const { return w_; }
  int height() const { return h_; }
  double resolution() const { return res_; }

  bool inBounds(int cx, int cy) const { return cx >= 0 && cx < w_ && cy >= 0 && cy < h_; }

  std::optional<Cell> worldToCell(double x, double y) const {
    // floor, not a cast: (int)(-0.3) == 0 but the point is in cell -1.
    const double fx = std::floor((x - ox_) / res_), fy = std::floor((y - oy_) / res_);
    if (fx < 0 || fy < 0 || fx >= w_ || fy >= h_) return std::nullopt;  // check before int cast
    return Cell{static_cast<int>(fx), static_cast<int>(fy)};
  }

  Vec2 cellCenter(int cx, int cy) const {
    idx(cx, cy);  // bounds check
    return {ox_ + (cx + 0.5) * res_, oy_ + (cy + 0.5) * res_};
  }

  double logOdds(int cx, int cy) const { return cells_[idx(cx, cy)]; }

  void update(int cx, int cy, double delta) {
    double& l = cells_[idx(cx, cy)];
    l = std::clamp(l + delta, kMinLogOdds, kMaxLogOdds);  // invariant: l in [min, max]
  }

  double probability(int cx, int cy) const { return toProb(logOdds(cx, cy)); }

  std::size_t countOccupied(double p_thresh) const {
    return static_cast<std::size_t>(std::count_if(
        cells_.begin(), cells_.end(), [p_thresh](double l) { return toProb(l) > p_thresh; }));
  }

 private:
  template <class T>
  static T checked(bool ok, T value, const char* what) {
    if (!ok) throw std::invalid_argument(what);
    return value;
  }
  static double toProb(double l) { return 1.0 - 1.0 / (1.0 + std::exp(l)); }

  // Single place that maps (cx, cy) -> storage index and enforces bounds.
  std::size_t idx(int cx, int cy) const {
    if (!inBounds(cx, cy)) throw std::out_of_range("cell out of bounds");
    return static_cast<std::size_t>(cy) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(cx);
  }

  int w_, h_;
  double res_, ox_, oy_;
  std::vector<double> cells_;  // row-major, log-odds
};
