# Thread-Safe Queue with Shutdown

The lidar driver thread produces scans; the odometry thread consumes them. Implement the queue between them.

```cpp
template <typename T>
class ThreadSafeQueue {
 public:
  explicit ThreadSafeQueue(std::size_t max_size = 0);  // 0 = unbounded
  bool push(T v);                  // false (and v dropped) if shut down
  std::optional<T> pop();          // BLOCKS until an item is available or the queue is shut down
  std::optional<T> tryPop();       // never blocks
  void shutdown();                 // wake all waiting consumers; further pushes are rejected
  std::size_t size() const;
};
```

### Semantics
- FIFO order.
- `pop()` returns `std::nullopt` **only** when the queue is shut down **and** empty. Items pushed before
  `shutdown()` are still drained.
- **Bounded mode** (`max_size > 0`): when full, `push` drops the **oldest** item to make room. Real-time
  consumers want the freshest scan, not a backlog.
- Many producers and many consumers may call concurrently. Tests run under sanitizers, so no data races
  and no deadlocks.

### Think about
- Why must `cv.wait` take a predicate? (spurious wakeups, lost wakeups)
- Notify inside or outside the lock: does it matter?
- Why must you never hold this lock while running ICP on the scan? What about priority inversion?
- Bounded queue: block the producer, or drop oldest? Which is right for a sensor driver?
