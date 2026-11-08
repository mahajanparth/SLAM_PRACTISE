#include <array>
#include <cstddef>
#include <optional>
#include <utility>

template <typename T, std::size_t N>
class RingBuffer {
  static_assert(N > 0, "RingBuffer capacity must be positive");

 public:
  void push(const T& v) { slotForPush() = v; }
  void push(T&& v) { slotForPush() = std::move(v); }  // move-only T (unique_ptr) works

  std::optional<T> pop() {
    if (size_ == 0) return std::nullopt;
    std::optional<T> out(std::move(buf_[head_]));
    buf_[head_] = T{};  // release resources held by the moved-from slot
    head_ = (head_ + 1) % N;
    --size_;
    return out;
  }

  T& operator[](std::size_t i) { return buf_[(head_ + i) % N]; }
  const T& operator[](std::size_t i) const { return buf_[(head_ + i) % N]; }
  const T& front() const { return buf_[head_]; }
  const T& back() const { return buf_[(head_ + size_ - 1) % N]; }
  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  bool full() const { return size_ == N; }
  void clear() {
    for (auto& x : buf_) x = T{};
    head_ = size_ = 0;
  }
  static constexpr std::size_t capacity() { return N; }

 private:
  // Returns the slot the next pushed element goes into, updating head_/size_.
  T& slotForPush() {
    if (size_ < N) return buf_[(head_ + size_++) % N];
    T& oldest = buf_[head_];  // full: overwrite the oldest, which makes the next one the oldest
    head_ = (head_ + 1) % N;
    return oldest;
  }

  std::array<T, N> buf_{};  // inline storage, no heap
  std::size_t head_ = 0;    // index of the oldest element
  std::size_t size_ = 0;    // size_ disambiguates full vs empty (head == tail in both)
};

template <typename T, std::size_t N>
T average(const RingBuffer<T, N>& rb) {
  if (rb.empty()) return T{};
  T acc{};
  for (std::size_t i = 0; i < rb.size(); ++i) acc = acc + rb[i];
  return acc / static_cast<double>(rb.size());
}
