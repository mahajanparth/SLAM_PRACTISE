# IMU Callback Registry

Your IMU driver publishes samples at 400 Hz to whoever subscribed: the EKF, a logger, a bias estimator. Implement the dispatcher:
```cpp
struct ImuSample { double t; double ax, ay, az; };

class SensorBus {
 public:
  using Callback = std::function<void(const ImuSample&)>;
  int subscribe(Callback cb);
  template <class T> int subscribeMember(T* obj, void (T::*method)(const ImuSample&));
  bool unsubscribe(int id);
  std::size_t publish(const ImuSample& s);
  std::size_t size() const;
};
```

| Member | Behaviour |
|---|---|
| `subscribe(cb)` | Registers `cb` and returns a **unique id** `1, 2, 3, …`. Ids are never reused, even after unsubscribing. An **empty** `cb` is rejected: return `-1` and register nothing. |
| `subscribeMember(obj, method)` | Registers a callback that calls `(obj->*method)(s)`. Returns the id, or `-1` if `obj` or `method` is null. The bus does **not** own `obj`. |
| `unsubscribe(id)` | Removes the callback. Returns `false` if the id is unknown (or already removed). |
| `publish(s)` | Calls every registered callback **in subscription order** and returns how many were called. |
| `size()` | The number of registered callbacks. |

### Re-entrancy rules (the interview part)
Callbacks may call `subscribe` / `unsubscribe` on the same bus **during** `publish`:
- A callback removed during a publish that **hasn't run yet** in that publish must **not** be called.
- A callback may unsubscribe **itself** while it's running. That must be memory-safe (tests run under ASan).
- A callback added during a publish is **not** called in that publish, only in later ones.

### Constraints
- Up to 10⁴ subscribers. `subscribe` / `unsubscribe` are O(log n), and `publish` is O(n log n) or better.
- Single-threaded (no locking needed).

### Think about
- What breaks if you store callbacks in a `std::vector` and erase while iterating?
- How would you make subscriptions unsubscribe automatically (RAII handle)? What if the bus dies first?
- `subscribeMember` stores a raw `T*`. What happens if the object is destroyed first? How would `std::weak_ptr` help?
- How would you make `publish` thread-safe without holding a lock while calling user code?
