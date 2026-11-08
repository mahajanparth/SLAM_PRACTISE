// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  {
    auto d = makeDriver("imu");
    CHECK(d && d->name() == "imu");
    CHECK(makeDriver("radar") == nullptr);
    auto ds = makeDrivers({"lidar", "radar", "camera", "imu"});
    CHECK(ds.size() == 3 && ds[0] && ds[0]->name() == "lidar");
    CHECK_NEAR(totalRate(ds), 440.0, 1e-12);
    auto fast = takeFastest(ds);
    CHECK(fast && fast->name() == "imu" && ds.size() == 2);
  }
  CHECK_EQ(g_live_drivers, 0);
  {
    DevicePtr dev = openDevice(3);
    CHECK(dev && dev->port == 3 && g_open_devices == 1);
  }
  CHECK_EQ(g_open_devices, 0);

#ifdef HIDDEN
  CHECK(makeDrivers({}).empty());
  CHECK(makeDriver("") == nullptr && makeDriver("Lidar") == nullptr);
  {
    std::vector<std::unique_ptr<Driver>> none;
    CHECK(takeFastest(none) == nullptr);
    auto ds = makeDrivers({"camera", "lidar", "camera"});
    auto f1 = takeFastest(ds);
    CHECK(f1 && f1->name() == "camera" && ds.size() == 2 && ds[0] && ds[0]->name() == "lidar");
    CHECK_EQ(g_live_drivers, 3);
    std::shared_ptr<Driver> shared = std::move(f1);  // unique -> shared upgrade
    CHECK(!f1 && shared && g_live_drivers == 3);
    ds.clear();
    CHECK_EQ(g_live_drivers, 1);
  }
  CHECK_EQ(g_live_drivers, 0);
  // devices: failure, reset, release, shared ownership
  CHECK(!openDevice(-1) && !openDevice(16));
  {
    DevicePtr a = openDevice(5);
    CHECK(a && !openDevice(5));  // already open
    a.reset();
    CHECK_EQ(g_open_devices, 0);
    DevicePtr b = openDevice(5);
    DevicePtr c = std::move(b);
    CHECK(!b && c && g_open_devices == 1);
  }
  CHECK_EQ(g_open_devices, 0);
  CHECK(openShared(99) == nullptr);
  {
    auto s1 = openShared(7);
    CHECK(s1 && g_open_devices == 1);
    {
      auto s2 = s1;
      s1.reset();
      CHECK(g_open_devices == 1 && s2.use_count() == 1);
    }
    CHECK_EQ(g_open_devices, 0);
  }
  CHECK(sizeof(DevicePtr) == sizeof(DeviceHandle*));
#endif
  return TEST_SUMMARY();
}
