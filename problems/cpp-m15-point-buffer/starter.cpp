#include <algorithm>
#include <cstddef>
#include <utility>

struct Point3 { double x, y, z; };

class PointBuffer {
 public:
  static inline int copies = 0;
  static inline int moves = 0;

  PointBuffer() = default;
  explicit PointBuffer(std::size_t n, Point3 fill = {0, 0, 0}) {
    // TODO
  }
  ~PointBuffer() {
    // TODO
  }
  PointBuffer(const PointBuffer& o) {
    // TODO
  }
  PointBuffer& operator=(const PointBuffer& o) {
    // TODO
    return *this;
  }
  // TODO: think about the exception specification of the move operations
  PointBuffer(PointBuffer&& o) {
    // TODO
  }
  PointBuffer& operator=(PointBuffer&& o) {
    // TODO
    return *this;
  }

  std::size_t size() const { return size_; }
  const Point3* data() const { return data_; }
  Point3& operator[](std::size_t i) { return data_[i]; }
  const Point3& operator[](std::size_t i) const { return data_[i]; }

 private:
  Point3* data_ = nullptr;
  std::size_t size_ = 0;
};
