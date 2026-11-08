# Sensor Message Bus with `std::variant`

Your front-end receives a mixed stream of messages. Instead of a polymorphic base class with heap allocation per
message, the bus uses a closed set of value types:

```cpp
struct LidarMsg { double stamp; std::vector<float> ranges; };
struct ImuMsg   { double stamp; double gyro_z; double acc_x; };
struct OdomMsg  { double stamp; double x, y, yaw; };
using Msg = std::variant<LidarMsg, ImuMsg, OdomMsg>;

struct Summary {
  int lidar = 0, imu = 0, odom = 0;   // message counts
  std::size_t valid_ranges = 0;       // lidar ranges that are finite and > 0
  double yaw_from_gyro = 0.0;         // ∫ gyro_z dt (see below)
  std::optional<OdomMsg> last_odom;   // odometry message with the largest stamp
};
```

Implement:

| Function | Behaviour |
|---|---|
| `double stampOf(const Msg& m)` | Timestamp of whichever message is held |
| `template <class T> std::vector<T> extract(const std::vector<Msg>& msgs)` | All messages of type `T`, in input order |
| `void sortByStamp(std::vector<Msg>& msgs)` | Sort by stamp; messages with **equal** stamps keep their relative order |
| `Summary process(const std::vector<Msg>& msgs)` | Fill a `Summary` in a single pass (input is sorted by stamp) |
| `std::optional<OdomMsg> latestOdomBefore(const std::vector<Msg>& msgs, double t)` | Odometry with the largest stamp `<= t` (input in any order); `nullopt` if none. For equal stamps, return the first. |

**Gyro integration** (rectangle rule, using the *earlier* sample): for consecutive IMU messages $i-1, i$ in the stream,
add `gyro_z[i-1] * (stamp[i] - stamp[i-1])`. Fewer than 2 IMU messages → 0. Other message types in between
don't matter. If several odometry messages share the largest stamp, `last_odom` is the last one in the stream.

Use `std::visit` (with `if constexpr` or the overloaded-lambdas idiom) or `std::get_if`. Never use `std::get` with a guessed type.

### Think about
- `sizeof(Msg)` vs `sizeof(LidarMsg)`? Where do the ranges live?
- What happens at compile time if someone adds `GpsMsg` to the variant but `process` uses the overloaded-lambdas idiom? And with a generic `else` branch?
- variant vs `std::unique_ptr<MsgBase>` with virtual `handle()`: allocation, cache behaviour, extensibility.
- Why does `sortByStamp` need a *stable* sort?
