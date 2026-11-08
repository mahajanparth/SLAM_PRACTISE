# Sensor Driver Factory

The robot's bring-up code reads a list of sensor kinds and creates drivers. Drivers are polymorphic (given in the starter: `Driver` with `LidarDriver` 10 Hz, `ImuDriver` 400 Hz, `CameraDriver` 30 Hz). Ownership must be explicit, and nothing may leak. The global `g_live_drivers` counts live driver objects.

There is also a C-style device API with a fixed table of 16 ports (given, do not modify):
```cpp
DeviceHandle* dev_open(int port);   // nullptr if port ∉ [0,16) or already open
void dev_close(DeviceHandle* h);    // closes; g_open_devices counts open handles
```

Implement:

| Function | Behaviour |
|---|---|
| `std::unique_ptr<Driver> makeDriver(const std::string& kind)` | `"lidar"`, `"imu"` or `"camera"` gives the matching driver. Anything else gives `nullptr`. |
| `std::vector<std::unique_ptr<Driver>> makeDrivers(const std::vector<std::string>& kinds)` | Drivers for all **known** kinds, in input order (unknown kinds are skipped). |
| `double totalRate(const std::vector<std::unique_ptr<Driver>>& ds)` | Sum of `rateHz()` (does not take ownership). |
| `std::unique_ptr<Driver> takeFastest(std::vector<std::unique_ptr<Driver>>& ds)` | **Moves out** the highest-rate driver (the first one if tied) and erases its slot from `ds`. `nullptr` if `ds` is empty. |
| `struct DeviceCloser` | Deleter so that `DevicePtr = std::unique_ptr<DeviceHandle, DeviceCloser>` closes the device. |
| `DevicePtr openDevice(int port)` | An owning `DevicePtr` (empty if the open failed). |
| `std::shared_ptr<DeviceHandle> openShared(int port)` | A shared owner that closes the device when the **last** copy dies. Empty if the open failed. |

### Constraints
- No leaks and no double-closes (ASan is on; the tests also check `g_live_drivers` and `g_open_devices`).
- `totalRate` must not copy or transfer ownership.

### Think about
- Why return `unique_ptr` from a factory rather than `shared_ptr`? What can the caller do with it?
- Why must `Driver` have a **virtual** destructor here?
- Why is `DeviceCloser` part of `DevicePtr`'s type, while the `shared_ptr`'s deleter is not? What does each cost in size?
- What does `sizeof(std::unique_ptr<DeviceHandle, void(*)(DeviceHandle*)>)` become, and why is an empty functor deleter better?
