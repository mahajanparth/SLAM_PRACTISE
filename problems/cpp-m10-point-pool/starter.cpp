#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

struct Point3 { double x, y, z; };

std::uintptr_t alignUp(std::uintptr_t addr, std::size_t align) {
  // TODO
  return 0;
}

class PointPool {
 public:
  explicit PointPool(std::size_t block_size) {
    // TODO
  }
  ~PointPool() {
    // TODO (if you manage memory manually)
  }
  PointPool(const PointPool&) = delete;
  PointPool& operator=(const PointPool&) = delete;

  Point3* allocate(double x, double y, double z) {
    // TODO
    return nullptr;
  }
  void deallocate(Point3* p) {
    // TODO
  }
  std::size_t size() const { return 0; /* TODO */ }
  std::size_t capacity() const { return 0; /* TODO */ }
  std::size_t blockCount() const { return 0; /* TODO */ }

 private:
  // TODO: your members
};
