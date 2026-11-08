#include <algorithm>
#include <cstddef>
#include <functional>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

struct Point3 { double x, y, z; };
struct Stamped { double t; int sensor; };

std::vector<int> kNearest(const std::vector<Point3>& cloud, const Point3& q, int k) {
  if (k <= 0) return {};
  // Max-heap on (squared distance, index): top() is the WORST of the best-k so far.
  // Pair comparison is lexicographic, so ties are broken by index for free.
  std::priority_queue<std::pair<double, int>> heap;
  const std::size_t kk = static_cast<std::size_t>(k);
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const double dx = cloud[i].x - q.x, dy = cloud[i].y - q.y, dz = cloud[i].z - q.z;
    const std::pair<double, int> cand{dx * dx + dy * dy + dz * dz, static_cast<int>(i)};
    if (heap.size() < kk) {
      heap.push(cand);
    } else if (cand < heap.top()) {  // better than the current worst -> replace it
      heap.pop();
      heap.push(cand);
    }
  }
  std::vector<int> out;
  out.reserve(heap.size());
  while (!heap.empty()) { out.push_back(heap.top().second); heap.pop(); }  // worst ... best
  std::reverse(out.begin(), out.end());
  return out;
}

std::vector<Stamped> mergeStreams(const std::vector<std::vector<Stamped>>& streams) {
  // (t, stream index, position in stream); std::greater<> -> min-heap, ties by stream then position.
  using Item = std::tuple<double, std::size_t, std::size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> heap;
  std::size_t total = 0;
  for (std::size_t s = 0; s < streams.size(); ++s) {
    total += streams[s].size();
    if (!streams[s].empty()) heap.emplace(streams[s][0].t, s, 0);
  }
  std::vector<Stamped> out;
  out.reserve(total);
  while (!heap.empty()) {
    const auto [t, s, pos] = heap.top();
    heap.pop();
    out.push_back(streams[s][pos]);
    if (pos + 1 < streams[s].size()) heap.emplace(streams[s][pos + 1].t, s, pos + 1);  // heap holds ≤ S items
  }
  return out;
}
