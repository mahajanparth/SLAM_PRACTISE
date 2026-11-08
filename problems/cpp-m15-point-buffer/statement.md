# Rule of Five: an owning `PointBuffer`

A legacy driver API hands you raw arrays, so your team keeps a small owning buffer type instead of `std::vector`. Implement all five special members of:
```cpp
class PointBuffer {
 public:
  static inline int copies = 0;   // ++ in copy ctor AND copy assignment (once per call)
  static inline int moves  = 0;   // ++ in move ctor AND move assignment (once per call)
  PointBuffer() = default;                                   // empty: size 0, data nullptr
  explicit PointBuffer(std::size_t n, Point3 fill = {0, 0, 0});
  ~PointBuffer();
  PointBuffer(const PointBuffer& o);
  PointBuffer& operator=(const PointBuffer& o);
  PointBuffer(PointBuffer&& o) /* ??? */;
  PointBuffer& operator=(PointBuffer&& o) /* ??? */;
  std::size_t size() const;  const Point3* data() const;  Point3& operator[](std::size_t i);  // ...
 private:
  Point3* data_ = nullptr;
  std::size_t size_ = 0;
};
```

| Member | Required semantics |
|---|---|
| `PointBuffer(n, fill)` | Owns `n` points, all equal to `fill`. `n == 0` gives the empty state (no allocation). |
| copy ctor / copy `=` | **Deep** copy: independent storage. Self-assignment is safe. If the allocation throws, the target is unchanged. |
| move ctor / move `=` | **Steal** the source's array (same address afterwards, no allocation). The source is left **empty** (`size()==0`, `data()==nullptr`). Self-move-assignment is a no-op. |
| `~PointBuffer()` | Releases the array. |

Each copy or move operation increments its counter **exactly once** per call (tests use the counters).

### Constraints
- ASan is on: no leaks, double frees or use-after-free.
- `std::vector<PointBuffer>` must **move** (never copy) its elements when it grows.

### Think about
- Why does `std::vector` copy instead of move during reallocation if your move constructor isn't `noexcept`?
- Copy-and-swap versus a hand-written copy assignment: which exception guarantee does each give, and what does copy-and-swap cost?
- Could this class follow the **rule of zero** instead? What would you change?
- What does "valid but unspecified" mean for a moved-from object, and why do we promise *empty* here?
