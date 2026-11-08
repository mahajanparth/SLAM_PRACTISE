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
  // TODO
  return nullptr;
}

std::vector<std::unique_ptr<Driver>> makeDrivers(const std::vector<std::string>& kinds) {
  // TODO
  return {};
}

double totalRate(const std::vector<std::unique_ptr<Driver>>& ds) {
  // TODO
  return 0.0;
}

std::unique_ptr<Driver> takeFastest(std::vector<std::unique_ptr<Driver>>& ds) {
  // TODO
  return nullptr;
}

struct DeviceCloser {
  void operator()(DeviceHandle* h) const {
    // TODO
  }
};
using DevicePtr = std::unique_ptr<DeviceHandle, DeviceCloser>;

DevicePtr openDevice(int port) {
  // TODO
  return DevicePtr{};
}

std::shared_ptr<DeviceHandle> openShared(int port) {
  // TODO
  return nullptr;
}
