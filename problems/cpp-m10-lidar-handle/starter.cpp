#include <cstdlib>
#include <stdexcept>
#include <utility>
#include <vector>

// ---------------- vendor C SDK (do not modify) ----------------
struct c_lidar { int port; int seq; };
inline int g_open_handles = 0;
inline c_lidar* c_lidar_open(int port) {
  if (port < 0) return nullptr;
  auto* h = static_cast<c_lidar*>(std::malloc(sizeof(c_lidar)));
  h->port = port; h->seq = 0;
  ++g_open_handles;
  return h;
}
inline int c_lidar_read(c_lidar* h, float* out, int n) {
  if (!h || !out) return 0;
  for (int i = 0; i < n; ++i) out[i] = float(h->port * 100 + h->seq) + 0.25f * float(i);
  ++h->seq;
  return n;
}
inline void c_lidar_close(c_lidar* h) {
  if (!h) return;
  --g_open_handles;
  std::free(h);
}
// ---------------------------------------------------------------

class LidarHandle {
 public:
  LidarHandle() = default;
  explicit LidarHandle(int port) {
    // TODO: open, throw std::runtime_error on failure
  }
  ~LidarHandle() {
    // TODO
  }
  LidarHandle(const LidarHandle&) = delete;
  LidarHandle& operator=(const LidarHandle&) = delete;

  LidarHandle(LidarHandle&& other) noexcept {
    // TODO
  }
  LidarHandle& operator=(LidarHandle&& other) noexcept {
    // TODO
    return *this;
  }

  bool valid() const { return false; /* TODO */ }
  c_lidar* get() const { return nullptr; /* TODO */ }
  c_lidar* release() {
    // TODO
    return nullptr;
  }
  void reset(c_lidar* h = nullptr) {
    // TODO
  }
  std::vector<float> read(int n) {
    // TODO
    return {};
  }

 private:
  c_lidar* h_ = nullptr;
};

std::vector<LidarHandle> openAll(const std::vector<int>& ports) {
  // TODO
  return {};
}
