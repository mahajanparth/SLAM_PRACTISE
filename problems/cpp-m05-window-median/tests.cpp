// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <algorithm>
#include <random>

static std::vector<double> brute(const std::vector<double>& x, std::size_t k) {
  std::vector<double> out;
  for (std::size_t i = 0; i < x.size(); ++i) {
    std::size_t s = i + 1 >= k ? i + 1 - k : 0;
    std::vector<double> w(x.begin() + s, x.begin() + i + 1);
    std::sort(w.begin(), w.end());
    std::size_t n = w.size();
    out.push_back(n % 2 ? w[n / 2] : 0.5 * (w[n / 2 - 1] + w[n / 2]));
  }
  return out;
}

int main() {
  RangeMedianFilter f(3);
  CHECK_NEAR(f.push(5.0), 5.0, 1e-12);
  CHECK_NEAR(f.push(1.0), 3.0, 1e-12);   // even: average
  CHECK_NEAR(f.push(100.0), 5.0, 1e-12); // spike rejected
  CHECK_NEAR(f.push(4.0), 4.0, 1e-12);   // window {1, 100, 4}
  CHECK_EQ(f.size(), std::size_t(3));

  std::vector<double> x = {2, 2, 9, 2, 3, 3, 50, 3};
  CHECK(medianFilter(x, 3) == brute(x, 3));

#ifdef HIDDEN
  CHECK(medianFilter({}, 5).empty());
  // k = 1: identity
  std::vector<double> id = {3, -1, 7, 7};
  CHECK(medianFilter(id, 1) == id);
  // k larger than the stream: running median of everything
  CHECK(medianFilter(id, 100) == brute(id, 100));
  // all duplicates: erase(value) would wipe the window
  std::vector<double> dup(50, 4.25);
  CHECK(medianFilter(dup, 7) == dup);
  // even k with duplicates crossing halves
  std::vector<double> mix = {1, 1, 1, 2, 2, 2, 1, 1, 2, 2, 3, 1};
  CHECK(medianFilter(mix, 4) == brute(mix, 4));
  // random quantised stream vs brute force, several k
  std::mt19937 g(7);
  std::uniform_int_distribution<int> u(0, 40);
  std::vector<double> r(4000);
  for (auto& v : r) v = u(g) * 0.25;
  CHECK(medianFilter(r, 2) == brute(r, 2));
  CHECK(medianFilter(r, 31) == brute(r, 31));
  CHECK(medianFilter(r, 64) == brute(r, 64));
  // large stream: O(log k) or O(k) per push is fine; re-sorting the window every push (O(k log k)) is too slow
  std::vector<double> big(100000);
  for (std::size_t i = 0; i < big.size(); ++i) big[i] = double((i * 2654435761u) % 1000);
  auto bm = medianFilter(big, 2001);
  CHECK(bm.size() == big.size());
  RangeMedianFilter g2(2001);
  double last = 0;
  for (std::size_t i = 0; i < big.size(); ++i) last = g2.push(big[i]);
  CHECK(last == bm.back() && g2.size() == 2001);
#endif
  return TEST_SUMMARY();
}
