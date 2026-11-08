#include <algorithm>
#include <cstddef>
#include <utility>

class FloatBuffer {
 public:
  FloatBuffer() {}
  explicit FloatBuffer(std::size_t n) {
    // TODO
  }
  ~FloatBuffer() {
    // TODO
  }
  FloatBuffer(const FloatBuffer& other) {
    // TODO
  }
  FloatBuffer& operator=(const FloatBuffer& other) {
    // TODO
    return *this;
  }
  FloatBuffer(FloatBuffer&& other) {
    // TODO
  }
  FloatBuffer& operator=(FloatBuffer&& other) {
    // TODO
    return *this;
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
