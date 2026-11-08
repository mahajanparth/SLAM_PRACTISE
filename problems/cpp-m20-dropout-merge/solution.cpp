#include <algorithm>
#include <vector>

struct Interval { double start, end; };

std::vector<Interval> mergeDropouts(std::vector<Interval> v, double max_gap) {
  if (v.empty()) return {};
  std::sort(v.begin(), v.end(), [](const Interval& a, const Interval& b) {
    return a.start < b.start || (a.start == b.start && a.end < b.end);
  });
  std::vector<Interval> out;
  out.push_back(v[0]);
  for (std::size_t i = 1; i < v.size(); ++i) {
    Interval& cur = out.back();
    if (v[i].start - cur.end <= max_gap) {       // overlap, touch, or small gap
      cur.end = std::max(cur.end, v[i].end);     // max: v[i] may be nested inside cur
    } else {
      out.push_back(v[i]);
    }
  }
  return out;
}

bool isDown(const std::vector<Interval>& merged, double t) {
  // First interval starting AFTER t; the candidate is the one before it.
  auto it = std::upper_bound(merged.begin(), merged.end(), t,
                             [](double x, const Interval& iv) { return x < iv.start; });
  if (it == merged.begin()) return false;
  --it;
  return t < it->end;  // half-open
}

double uptime(const std::vector<Interval>& merged, double t0, double t1) {
  if (t1 <= t0) return 0.0;
  double down = 0.0;
  for (const Interval& iv : merged) {
    if (iv.start >= t1) break;  // sorted: nothing further can overlap
    down += std::max(0.0, std::min(iv.end, t1) - std::max(iv.start, t0));
  }
  return (t1 - t0) - down;
}

std::vector<Interval> bothDown(const std::vector<Interval>& a, const std::vector<Interval>& b) {
  std::vector<Interval> out;
  std::size_t i = 0, j = 0;
  while (i < a.size() && j < b.size()) {
    const double s = std::max(a[i].start, b[j].start);
    const double e = std::min(a[i].end, b[j].end);
    if (s < e) out.push_back({s, e});
    // The interval that ends first can't overlap anything further in the other list.
    if (a[i].end < b[j].end) ++i; else ++j;
  }
  return out;
}
