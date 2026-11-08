#include <array>
#include <cstddef>
#include <stdexcept>

template <typename T, std::size_t N>
class RingBuffer {
  static_assert(N >= 1, "capacity must be positive");

 public:
  void push(const T& v) {
    // TODO
  }
  std::size_t size() const {
    // TODO
    return 0;
  }
  bool empty() const { return size() == 0; }
  bool full() const { return size() == N; }
  static constexpr std::size_t capacity() { return N; }

  const T& operator[](std::size_t i) const {
    // TODO: 0 = oldest; throw std::out_of_range if i >= size()
    return buf_[0];
  }
  const T& back() const {
    // TODO: newest; throw std::out_of_range if empty
    return buf_[0];
  }
  void clear() {
    // TODO
  }

 private:
  std::array<T, N> buf_{};
  // TODO: add your indices
};
