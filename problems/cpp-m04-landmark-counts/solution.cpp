#include <map>
#include <unordered_map>
#include <vector>

struct Observation { int frame_id; int landmark_id; };

std::unordered_map<int, int> countObservations(const std::vector<Observation>& obs) {
  std::unordered_map<int, int> counts;
  counts.reserve(obs.size());                  // upper bound on distinct keys: no rehash while counting
  for (const auto& o : obs) ++counts[o.landmark_id];  // [] value-initialises a missing key to 0
  return counts;
}

int countOf(const std::unordered_map<int, int>& counts, int id) {
  auto it = counts.find(id);                   // [] is non-const (it may insert), so use find
  return it == counts.end() ? 0 : it->second;
}

int mostObserved(const std::vector<Observation>& obs) {
  const auto counts = countObservations(obs);
  int best_id = -1, best = 0;
  for (const auto& [id, c] : counts)           // unordered: tie-break must be explicit
    if (c > best || (c == best && id < best_id)) { best = c; best_id = id; }
  return best_id;
}

std::vector<int> stableLandmarks(const std::vector<Observation>& obs, int min_count) {
  std::map<int, int> counts;                   // ordered by key (red-black tree)
  for (const auto& o : obs) ++counts[o.landmark_id];
  std::vector<int> out;
  for (const auto& [id, c] : counts)           // in-order traversal = ascending ids
    if (c >= min_count) out.push_back(id);
  return out;
}
