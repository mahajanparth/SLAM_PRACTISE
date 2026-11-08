#include <algorithm>
#include <cstddef>
#include <utility>

class FloatBuffer {
 public:
  FloatBuffer() = default;
  explicit FloatBuffer(std::size_t n) : data_(n ? new float[n]() : nullptr), size_(n) {}
  ~FloatBuffer() { delete[] data_; }  // delete[] nullptr is a no-op

  // Deep copy. If new[] throws, nothing has been modified yet.
  FloatBuffer(const FloatBuffer& other) : FloatBuffer(other.size_) {
    std::copy(other.data_, other.data_ + other.size_, data_);
  }

  // Steal the pointer and leave `other` empty but valid. noexcept, so std::vector moves on realloc.
  FloatBuffer(FloatBuffer&& other) noexcept
      : data_(std::exchange(other.data_, nullptr)), size_(std::exchange(other.size_, 0)) {}

  // Copy-and-swap: `other` is a copy (from an lvalue) or a moved-into value (from an rvalue).
  // Handles self-assignment and gives the strong exception guarantee; the old buffer is
  // released by other's destructor.
  FloatBuffer& operator=(FloatBuffer other) noexcept {
    swap(other);
    return *this;
  }

  void swap(FloatBuffer& other) noexcept {
    std::swap(data_, other.data_);
    std::swap(size_, other.size_);
  }

  float& operator[](std::size_t i) { return data_[i]; }
  const float& operator[](std::size_t i) const { return data_[i]; }
  std::size_t size() const { return size_; }
  float* data() { return data_; }
  const float* data() const { return data_; }

 private:
  float* data_ = nullptr;
  std::size_t size_ = 0;
};
