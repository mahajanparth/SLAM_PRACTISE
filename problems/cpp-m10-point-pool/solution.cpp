#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

struct Point3 { double x, y, z; };

std::uintptr_t alignUp(std::uintptr_t addr, std::size_t align) {
  // align is a power of two: add (align-1), then clear the low bits.
  const std::uintptr_t a = static_cast<std::uintptr_t>(align);
  return (addr + a - 1) & ~(a - 1);
}

class PointPool {
 public:
  explicit PointPool(std::size_t block_size) : block_size_(block_size ? block_size : 1) {}
  // Blocks are unique_ptr<Point3[]>, so the destructor is implicit (rule of zero for memory);
  // copying is still disabled because pointers handed out refer to *this* pool's blocks.
  ~PointPool() = default;
  PointPool(const PointPool&) = delete;
  PointPool& operator=(const PointPool&) = delete;

  Point3* allocate(double x, double y, double z) {
    Point3* slot;
    if (!free_.empty()) {  // LIFO reuse: the most recently freed slot is likely still in cache
      slot = free_.back();
      free_.pop_back();
    } else {
      if (blocks_.empty() || used_in_last_ == block_size_) {
        // A NEW block: existing blocks never move, so old pointers stay valid.
        blocks_.push_back(std::make_unique<Point3[]>(block_size_));
        used_in_last_ = 0;
      }
      slot = &blocks_.back()[used_in_last_++];
    }
    *slot = Point3{x, y, z};
    ++live_;
    return slot;
  }

  void deallocate(Point3* p) {
    if (!p) return;
    free_.push_back(p);  // Point3 is trivially destructible: nothing to destroy
    --live_;
  }

  std::size_t size() const { return live_; }
  std::size_t capacity() const { return blocks_.size() * block_size_; }
  std::size_t blockCount() const { return blocks_.size(); }

 private:
  std::size_t block_size_;
  std::vector<std::unique_ptr<Point3[]>> blocks_;  // owning; addresses inside are stable
  std::size_t used_in_last_ = 0;                   // slots handed out from the last block
  std::vector<Point3*> free_;                      // freed slots (non-owning)
  std::size_t live_ = 0;
};
