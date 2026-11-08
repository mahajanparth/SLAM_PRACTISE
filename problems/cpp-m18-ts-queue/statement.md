# Thread-Safe Sensor Queue

The lidar driver thread produces scans and the odometry thread consumes them. If the consumer falls behind, the
driver must **never block**. It drops the oldest scan instead, since stale data is worthless. On shutdown, every
blocked consumer must wake up and exit.

```cpp
template <typename T>
class ThreadSafeQueue {
 public:
  explicit ThreadSafeQueue(std::size_t capacity = 0);       // 0 = unbounded
  bool push(T v);                                            // see below
  std::optional<T> pop();                                    // BLOCKING
  std::optional<T> tryPop();                                 // non-blocking
  std::optional<T> popFor(std::chrono::milliseconds timeout);// blocks at most ~timeout
  void close();
  std::size_t size() const;
  bool closed() const;
  std::size_t dropped() const;                               // items discarded by drop-oldest
};
```

| Member | Semantics |
|---|---|
| `push(v)` | If closed: discard `v` and return `false`. If bounded and full: discard the **oldest** item (`++dropped`), then append. Returns `true`. Never blocks waiting for space |
| `pop()` | Wait until an item is available **or** the queue is closed. Returns the oldest item, or `nullopt` only when closed **and** empty |
| `tryPop()` | Oldest item if any, else `nullopt` immediately |
| `popFor(t)` | Like `pop()`, but gives up after `t` and returns `nullopt` |
| `close()` | Idempotent. Wakes **all** waiting consumers. Items already queued can still be popped |

FIFO order must hold. `T` may be move-only (e.g. `std::unique_ptr<Scan>`).

### Testing notes
The tests use real threads: several producers and consumers, and consumers blocked in `pop()` that must be woken by
`push` or `close`. A missed notification shows up as a **timeout**. Sleeps in the tests are ≤ 20 ms.

### Think about
- Why `notify_one` in `push` but `notify_all` in `close`?
- Why does the wait need a predicate? What is a lost wakeup?
- Should you call `notify` while holding the lock?
- Drop-oldest vs. block-producer vs. drop-newest: which fits a lidar driver, an IMU, a logging thread?
- Why can't `pop()` return `T&` or a reference to the front?
