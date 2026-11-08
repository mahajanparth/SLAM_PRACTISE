#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// ---------------- given: drivers ----------------
inline int g_live_drivers = 0;
struct Driver {
  Driver() { ++g_live_drivers; }
  Driver(const Driver&) = delete;
  Driver& operator=(const Driver&) = delete;
  virtual ~Driver() { --g_live_drivers; }
  virtual std::string name() const = 0;
  virtual double rateHz() const = 0;
};
struct LidarDriver : Driver {
  std::string name() const override { return "lidar"; }
  double rateHz() const override { return 10.0; }
};
struct ImuDriver : Driver {
  std::string name() const override { return "imu"; }
  double rateHz() const override { return 400.0; }
};
struct CameraDriver : Driver {
  std::string name() const override { return "camera"; }
  double rateHz() const override { return 30.0; }
};

// ---------------- given: C device API (do not modify) ----------------
struct DeviceHandle { int port; bool open; };
inline int g_open_devices = 0;
inline DeviceHandle g_device_table[16];
inline DeviceHandle* dev_open(int port) {
  if (port < 0 || port >= 16 || g_device_table[port].open) return nullptr;
  g_device_table[port] = {port, true};
  ++g_open_devices;
  return &g_device_table[port];
}
inline void dev_close(DeviceHandle* h) {
  if (!h || !h->open) return;
  h->open = false;
  --g_open_devices;
}
// -----------------------------------------------------------------------

std::unique_ptr<Driver> makeDriver(const std::string& kind) {
  // unique_ptr<Derived> -> unique_ptr<Base> converts implicitly; the virtual dtor makes
  // deleting through Driver* correct.
  if (kind == "lidar") return std::make_unique<LidarDriver>();
  if (kind == "imu") return std::make_unique<ImuDriver>();
  if (kind == "camera") return std::make_unique<CameraDriver>();
  return nullptr;
}

std::vector<std::unique_ptr<Driver>> makeDrivers(const std::vector<std::string>& kinds) {
  std::vector<std::unique_ptr<Driver>> out;
  out.reserve(kinds.size());
  for (const auto& k : kinds)
    if (auto d = makeDriver(k)) out.push_back(std::move(d));  // move-only: must std::move
  return out;  // vector of unique_ptr is movable, so this is cheap
}

double totalRate(const std::vector<std::unique_ptr<Driver>>& ds) {
  double total = 0.0;
  for (const auto& d : ds) total += d->rateHz();  // const auto&: no copy (copy wouldn't compile)
  return total;
}

std::unique_ptr<Driver> takeFastest(std::vector<std::unique_ptr<Driver>>& ds) {
  if (ds.empty()) return nullptr;
  // max_element returns the FIRST maximum with a strict < comparator.
  auto it = std::max_element(ds.begin(), ds.end(), [](const auto& a, const auto& b) {
    return a->rateHz() < b->rateHz();
  });
  std::unique_ptr<Driver> out = std::move(*it);  // transfer ownership; slot is now null
  ds.erase(it);
  return out;
}

struct DeviceCloser {
  void operator()(DeviceHandle* h) const { dev_close(h); }  // empty functor: zero size overhead
};
using DevicePtr = std::unique_ptr<DeviceHandle, DeviceCloser>;

DevicePtr openDevice(int port) { return DevicePtr(dev_open(port)); }

std::shared_ptr<DeviceHandle> openShared(int port) {
  DeviceHandle* h = dev_open(port);
  if (!h) return nullptr;
  // The deleter is stored (type-erased) in the control block; it runs when the LAST owner dies.
  return std::shared_ptr<DeviceHandle>(h, dev_close);
}
