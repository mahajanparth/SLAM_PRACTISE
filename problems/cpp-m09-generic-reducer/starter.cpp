#include <cmath>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

struct LidarPoint { double x, y, z; float intensity; int ring; };

template <class F, class G>
auto compose(F f, G g) {
  // TODO: return a generic lambda computing g(f(x))
  return [](auto&&) { return 0; };
}

template <class... Preds>
auto allOf(Preds... preds) {
  // TODO: return a generic lambda that is true iff every predicate holds
  return [](const auto&) { return false; };
}

template <class T, class KeyFn, class Acc, class Op>
std::map<std::decay_t<std::invoke_result_t<KeyFn, const T&>>, Acc>
groupReduce(const std::vector<T>& items, KeyFn key, Acc init, Op op) {
  // TODO
  return {};
}

inline auto makeRunningMean() {
  // TODO: return a stateful lambda double(double)
  return [](double) { return 0.0; };
}
