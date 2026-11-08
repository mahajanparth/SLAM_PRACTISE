#include <array>
#include <cstddef>
#include <stdexcept>

template <typename T, std::size_t N>
class RingBuffer {
  static_assert(N >= 1, "capacity must be positive");

 public:
  void push(const T& v) {
    buf_[head_] = v;              // head_ is always the next slot to write
    head_ = (head_ + 1) % N;      // when full this overwrites the oldest
    if (size_ < N) ++size_;
  }
  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  bool full() const { return size_ == N; }
  static constexpr std::size_t capacity() { return N; }

  const T& operator[](std::size_t i) const {
    if (i >= size_) throw std::out_of_range("RingBuffer index");
    // oldest element sits size_ slots behind head_; + N keeps it unsigned-safe
    return buf_[(head_ + N - size_ + i) % N];
  }
  const T& back() const {
    if (size_ == 0) throw std::out_of_range("RingBuffer empty");
    return buf_[(head_ + N - 1) % N];
  }
  void clear() { head_ = 0; size_ = 0; }

 private:
  std::array<T, N> buf_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};
