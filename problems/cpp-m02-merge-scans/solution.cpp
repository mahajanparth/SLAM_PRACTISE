#include <algorithm>
#include <cstddef>
#include <functional>
#include <queue>
#include <tuple>
#include <vector>

struct Point3 { double x, y, z; };
struct StampedPoint { double t; Point3 p; };

void appendScan(std::vector<Point3>& dst, const std::vector<Point3>& src) {
  // Capture the count first. If src IS dst, src.size() grows while we push.
  const std::size_t n = src.size();
  dst.reserve(dst.size() + n);  // the single (possible) reallocation happens here
  // Index, don't iterate: after reserve, any old iterators/references into src (== dst) dangle,
  // but src[i] re-reads the (possibly new) buffer each time.
  for (std::size_t i = 0; i < n; ++i) dst.push_back(src[i]);  // no reallocation: capacity suffices
}

std::vector<StampedPoint> mergeSorted(const std::vector<std::vector<StampedPoint>>& scans) {
  std::size_t total = 0;
  for (const auto& s : scans) total += s.size();
  std::vector<StampedPoint> out;
  out.reserve(total);  // exactly one allocation (none if total == 0)

  // k-way merge: the heap holds the current head of every non-empty scan.
  // Tuple order (t, scan, idx) gives the tie-break: earlier scan, then earlier element.
  using Head = std::tuple<double, std::size_t, std::size_t>;
  std::priority_queue<Head, std::vector<Head>, std::greater<>> heap;  // min-heap
  for (std::size_t k = 0; k < scans.size(); ++k)
    if (!scans[k].empty()) heap.emplace(scans[k][0].t, k, 0);

  while (!heap.empty()) {
    auto [t, k, i] = heap.top();
    heap.pop();
    out.push_back(scans[k][i]);
    if (i + 1 < scans[k].size()) heap.emplace(scans[k][i + 1].t, k, i + 1);
  }
  return out;  // O(N log k)
}
