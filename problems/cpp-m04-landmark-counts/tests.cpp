// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  std::vector<Observation> obs = {{0, 7}, {0, 3}, {1, 7}, {1, 5}, {2, 7}, {2, 3}, {2, 3}};
  auto counts = countObservations(obs);
  CHECK_EQ(counts.size(), std::size_t(3));
  CHECK_EQ(countOf(counts, 7), 3);
  CHECK_EQ(countOf(counts, 3), 3);
  CHECK_EQ(countOf(counts, 42), 0);
  CHECK_EQ(counts.size(), std::size_t(3));      // lookup did not insert
  CHECK_EQ(mostObserved(obs), 3);                // tie 3 vs 7 -> smaller id
  CHECK(stableLandmarks(obs, 2) == (std::vector<int>{3, 7}));

#ifdef HIDDEN
  CHECK_EQ(mostObserved({}), -1);
  CHECK(countObservations({}).empty());
  CHECK(stableLandmarks({}, 1).empty());
  // negative ids, ascending order with negatives
  std::vector<Observation> neg = {{0, -5}, {1, -5}, {1, 10}, {2, -100}, {3, 10}, {4, -100}};
  CHECK(stableLandmarks(neg, 2) == (std::vector<int>{-100, -5, 10}));
  CHECK_EQ(mostObserved(neg), -100);
  CHECK(stableLandmarks(neg, 3).empty());
  CHECK(stableLandmarks(neg, 0).size() == 3);
  // single observation
  CHECK_EQ(mostObserved({{9, 4}}), 4);
  // large: 200k observations over 1000 landmarks; landmark 777 gets a few extra
  std::vector<Observation> big;
  for (int i = 0; i < 200000; ++i) big.push_back({i / 100, (i * 7919) % 1000});
  for (int k = 0; k < 3; ++k) big.push_back({5000, 777});
  auto bc = countObservations(big);
  CHECK_EQ(bc.size(), std::size_t(1000));
  CHECK_EQ(countOf(bc, 777), 203);
  CHECK_EQ(mostObserved(big), 777);
  CHECK(stableLandmarks(big, 201) == (std::vector<int>{777}));
#endif
  return TEST_SUMMARY();
}
