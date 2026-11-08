# RAII Handle for a C Lidar SDK

The vendor SDK for your lidar is a C library (already defined in the starter; do not modify it):
```cpp
c_lidar* c_lidar_open(int port);                       // nullptr on failure (port < 0); allocates
int      c_lidar_read(c_lidar* h, float* out, int n);  // fills n ranges, returns n
void     c_lidar_close(c_lidar* h);                    // frees; accepts nullptr
inline int g_open_handles;                             // number of currently open handles (test hook)
```
Every handle that's opened must be closed **exactly once**, on every path, including exceptions. Write the RAII wrapper:

| Member | Behaviour |
|---|---|
| `LidarHandle()` | An empty handle (`valid() == false`). |
| `explicit LidarHandle(int port)` | Opens the device. Throws `std::runtime_error` if `c_lidar_open` fails. |
| `~LidarHandle()` | Closes the device if one is held. |
| copy ctor / copy assign | **Deleted** (already in the starter). |
| `LidarHandle(LidarHandle&&) noexcept`, `operator=(LidarHandle&&) noexcept` | Transfer ownership. The source becomes empty. Move-assign closes the device the target held before. Self-move is a no-op. |
| `bool valid() const`, `c_lidar* get() const` | Observe the handle. |
| `c_lidar* release()` | Give up ownership **without** closing, and return the raw handle. |
| `void reset(c_lidar* h = nullptr)` | Close the current device (if any, and if different from `h`), then own `h`. |
| `std::vector<float> read(int n)` | Read `n` ranges. Throws `std::logic_error` if not valid. Returns `{}` if `n ≤ 0`. |

And a free function:

| Function | Behaviour |
|---|---|
| `std::vector<LidarHandle> openAll(const std::vector<int>& ports)` | Open all ports in order. **All-or-nothing:** if any open fails, the exception propagates and **no** device stays open. |

### Constraints
- No leaks, no double-closes: the tests check `g_open_handles` and run under ASan with leak detection.
- `std::vector<LidarHandle>` must be able to grow (so moves must be `noexcept`).

### Think about
- Why does `std::vector` copy instead of move during reallocation if your move constructor isn't `noexcept`? Here copy is deleted, so what happens then?
- This is `std::unique_ptr<c_lidar, Closer>` in disguise. Write that one-liner.
- What does "strong exception guarantee" mean for `openAll`?
- Why can't a destructor report a failed `close`? What do real drivers do?
