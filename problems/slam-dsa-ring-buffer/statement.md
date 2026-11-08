# Ring Buffer for IMU Samples

An IMU driver pushes samples at 400 Hz. The estimator only needs the most recent `N` samples, and
allocating on every push is not allowed in the real-time thread. Implement a fixed-capacity ring buffer
that **overwrites the oldest** element when full.

```cpp
template <typename T, std::size_t N>
class RingBuffer {
 public:
  void push(const T& v);              // append; if full, overwrite the oldest
  std::size_t size() const;           // number of stored elements (<= N)
  bool empty() const;
  bool full() const;
  static constexpr std::size_t capacity() { return N; }
  const T& operator[](std::size_t i) const;  // i = 0 is the OLDEST, size()-1 the newest
  const T& back() const;              // newest element
  void clear();
};
```

### Requirements
- No heap allocation (`std::array` storage). All operations O(1).
- `operator[](i)` with `i >= size()` and `back()` on an empty buffer throw `std::out_of_range`.
- `N >= 1`. `N = 1` must work: it always holds only the last pushed value.

### Think about
- How would you make this a lock-free single-producer/single-consumer queue? (`std::atomic<size_t>` head/tail, acquire/release ordering)
- Why do many implementations require `N` to be a power of two? (`& (N-1)` instead of `%`)
- What does `T` need to support? What changes if `T` is not default-constructible?
