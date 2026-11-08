#include <algorithm>
#include <cstddef>
#include <utility>

struct Point3 { double x, y, z; };

class PointBuffer {
 public:
  static inline int copies = 0;
  static inline int moves = 0;

  PointBuffer() = default;
  explicit PointBuffer(std::size_t n, Point3 fill = {0, 0, 0})
      : data_(n ? new Point3[n] : nullptr), size_(n) {
    std::fill(data_, data_ + size_, fill);
  }

  ~PointBuffer() { delete[] data_; }  // delete[] nullptr is fine

  PointBuffer(const PointBuffer& o) : PointBuffer(o.data_, o.size_) { ++copies; }

  // Copy-and-swap: the copy is made BEFORE we touch *this, so a throwing `new` leaves us
  // unchanged (strong guarantee), and `a = a` just copies then swaps (safe).
  PointBuffer& operator=(const PointBuffer& o) {
    PointBuffer tmp(o.data_, o.size_);  // private deep-copy ctor: doesn't bump `copies`
    swap(tmp);
    ++copies;
    return *this;  // tmp's destructor frees our old array
  }

  // noexcept is essential: std::vector only moves during reallocation if the move ctor
  // can't throw (std::move_if_noexcept); otherwise it falls back to copying.
  PointBuffer(PointBuffer&& o) noexcept
      : data_(std::exchange(o.data_, nullptr)), size_(std::exchange(o.size_, 0)) {
    ++moves;
  }

  PointBuffer& operator=(PointBuffer&& o) noexcept {
    if (this != &o) {
      delete[] data_;
      data_ = std::exchange(o.data_, nullptr);
      size_ = std::exchange(o.size_, 0);
      ++moves;
    }
    return *this;
  }

  void swap(PointBuffer& o) noexcept {
    std::swap(data_, o.data_);
    std::swap(size_, o.size_);
  }

  std::size_t size() const { return size_; }
  const Point3* data() const { return data_; }
  Point3& operator[](std::size_t i) { return data_[i]; }
  const Point3& operator[](std::size_t i) const { return data_[i]; }

 private:
  // Deep-copy helper shared by the copy ctor and copy assignment.
  PointBuffer(const Point3* src, std::size_t n) : data_(n ? new Point3[n] : nullptr), size_(n) {
    std::copy(src, src + n, data_);
  }

  Point3* data_ = nullptr;
  std::size_t size_ = 0;
};
