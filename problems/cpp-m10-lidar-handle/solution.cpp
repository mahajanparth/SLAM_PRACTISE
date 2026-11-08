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
  explicit LidarHandle(int port) : h_(c_lidar_open(port)) {
    // Throwing from a constructor is fine: the object never existed, so the destructor
    // won't run, and nothing was acquired.
    if (!h_) throw std::runtime_error("c_lidar_open failed");
  }
  ~LidarHandle() { c_lidar_close(h_); }  // the SDK accepts nullptr

  // Unique ownership: a copy would close the same device twice.
  LidarHandle(const LidarHandle&) = delete;
  LidarHandle& operator=(const LidarHandle&) = delete;

  LidarHandle(LidarHandle&& other) noexcept : h_(std::exchange(other.h_, nullptr)) {}
  LidarHandle& operator=(LidarHandle&& other) noexcept {
    if (this != &other) reset(other.release());  // close ours, take theirs
    return *this;
  }

  bool valid() const { return h_ != nullptr; }
  c_lidar* get() const { return h_; }
  c_lidar* release() { return std::exchange(h_, nullptr); }
  void reset(c_lidar* h = nullptr) {
    if (h == h_) return;  // reset(get()) must not close the device we keep
    c_lidar_close(std::exchange(h_, h));
  }

  std::vector<float> read(int n) {
    if (!h_) throw std::logic_error("read on invalid LidarHandle");
    if (n <= 0) return {};
    std::vector<float> out(static_cast<std::size_t>(n));  // RAII buffer, not new float[n]
    c_lidar_read(h_, out.data(), n);
    return out;
  }

 private:
  c_lidar* h_ = nullptr;
};

std::vector<LidarHandle> openAll(const std::vector<int>& ports) {
  std::vector<LidarHandle> out;
  out.reserve(ports.size());
  // If any constructor throws, `out` is destroyed during unwinding and closes every
  // handle opened so far: all-or-nothing for free.
  for (int p : ports) out.emplace_back(p);
  return out;
}
