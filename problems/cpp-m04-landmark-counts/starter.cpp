#include <map>
#include <unordered_map>
#include <vector>

struct Observation { int frame_id; int landmark_id; };

std::unordered_map<int, int> countObservations(const std::vector<Observation>& obs) {
  // TODO
  return {};
}

int countOf(const std::unordered_map<int, int>& counts, int id) {
  // TODO: don't insert
  return -1;
}

int mostObserved(const std::vector<Observation>& obs) {
  // TODO
  return -2;
}

std::vector<int> stableLandmarks(const std::vector<Observation>& obs, int min_count) {
  // TODO
  return {};
}
