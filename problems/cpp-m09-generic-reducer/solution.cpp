#include <cmath>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

struct LidarPoint { double x, y, z; float intensity; int ring; };

template <class F, class G>
auto compose(F f, G g) {
  // Capture by value (moved in): the result must not refer to the factory's parameters.
  // `auto&&` + forward makes h perfectly forward to f.
  return [f = std::move(f), g = std::move(g)](auto&& x) {
    return g(f(std::forward<decltype(x)>(x)));
  };
}

template <class... Preds>
auto allOf(Preds... preds) {
  // C++17 fold over &&: short-circuits left to right; the empty fold is `true`.
  return [preds...](const auto& x) { return (true && ... && preds(x)); };
}

template <class T, class KeyFn, class Acc, class Op>
std::map<std::decay_t<std::invoke_result_t<KeyFn, const T&>>, Acc>
groupReduce(const std::vector<T>& items, KeyFn key, Acc init, Op op) {
  std::map<std::decay_t<std::invoke_result_t<KeyFn, const T&>>, Acc> out;
  for (const T& item : items) {
    // try_emplace only copies `init` the first time a key is seen.
    Acc& acc = out.try_emplace(key(item), init).first->second;
    acc = op(acc, item);
  }
  return out;
}

inline auto makeRunningMean() {
  // Init-captures create the closure's state; `mutable` lets operator() modify it.
  // Incremental (Welford-style) mean: no unbounded sum, numerically stable.
  return [n = 0LL, mean = 0.0](double x) mutable {
    ++n;
    mean += (x - mean) / static_cast<double>(n);
    return mean;
  };
}
