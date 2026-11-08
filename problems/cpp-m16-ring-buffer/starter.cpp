#include <array>
#include <cstddef>
#include <optional>
#include <utility>

template <typename T, std::size_t N>
class RingBuffer {
  static_assert(N > 0, "RingBuffer capacity must be positive");

 public:
  void push(const T& v) {
    // TODO
  }
  void push(T&& v) {
    // TODO (move v in)
  }
  std::optional<T> pop() {
    // TODO
    return std::nullopt;
  }
  T& operator[](std::size_t i) {
    // TODO
    return buf_[0];
  }
  const T& operator[](std::size_t i) const {
    // TODO
    return buf_[0];
  }
  const T& front() const { return buf_[0]; /* TODO */ }
  const T& back() const { return buf_[0]; /* TODO */ }
  std::size_t size() const { return 0; /* TODO */ }
  bool empty() const { return true; /* TODO */ }
  bool full() const { return false; /* TODO */ }
  void clear() { /* TODO */ }
  static constexpr std::size_t capacity() { return N; }

 private:
  std::array<T, N> buf_{};
  std::size_t head_ = 0;  // index of the oldest element
  std::size_t size_ = 0;
};

template <typename T, std::size_t N>
T average(const RingBuffer<T, N>& rb) {
  // TODO
  return T{};
}
