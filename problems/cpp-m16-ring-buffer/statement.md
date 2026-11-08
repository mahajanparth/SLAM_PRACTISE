# Generic `RingBuffer<T, N>`

An IMU driver keeps the last `N` samples in a fixed-capacity circular buffer. Pre-integration reads them oldest-first.
Allocating on every sample is not allowed at 1 kHz, so storage must be a `std::array<T, N>` member.

Implement the class template (skeleton given):

```cpp
template <typename T, std::size_t N>
class RingBuffer {
 public:
  void push(const T& v);          // append newest; if full, OVERWRITE the oldest
  void push(T&& v);               // same, but moves (must work for move-only T)
  std::optional<T> pop();         // remove and return the OLDEST; std::nullopt if empty
  T& operator[](std::size_t i);   // i = 0 is the oldest, size()-1 the newest (no bounds check)
  const T& operator[](std::size_t i) const;
  const T& front() const;         // oldest  (precondition: !empty())
  const T& back() const;          // newest  (precondition: !empty())
  std::size_t size() const;  bool empty() const;  bool full() const;  void clear();
  static constexpr std::size_t capacity() { return N; }
};

template <typename T, std::size_t N>
T average(const RingBuffer<T, N>& rb);   // mean of the elements; T{} if empty
```
`average` must work for any `T` that supports `T{}`, `T + T`, and `T / double`. The tests use `double` and a small `Vec3` struct.

### Constraints
- All operations O(1) (except `average` and `clear`, which may be O(N)).
- No heap allocation inside the buffer itself. `N == 0` should fail to compile (`static_assert`).

### Think about
- `head_` + `size_` vs. `head_` + `tail_`: how do you tell "full" from "empty"?
- Why does `pop()` return `std::optional<T>` rather than `T&`?
- What does `buf_{}` require of `T`? How would you support types without a default constructor?
- Is this buffer thread-safe? What would a lock-free SPSC version need?
