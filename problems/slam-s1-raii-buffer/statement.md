# RAII `FloatBuffer`: Rule of Five

A lidar driver hands you scans in raw `float` buffers. Write a small owning wrapper **without** using `std::vector` or smart pointers inside, so you have to get the special member functions right yourself.

```cpp
class FloatBuffer {
 public:
  FloatBuffer();                                   // empty: size() == 0, data() == nullptr
  explicit FloatBuffer(std::size_t n);             // n zero-initialised floats (n == 0 -> empty)
  ~FloatBuffer();
  FloatBuffer(const FloatBuffer& other);           // deep copy
  FloatBuffer& operator=(const FloatBuffer& other);
  FloatBuffer(FloatBuffer&& other);                // steal; other becomes empty
  FloatBuffer& operator=(FloatBuffer&& other);
  float& operator[](std::size_t i);
  const float& operator[](std::size_t i) const;
  std::size_t size() const;
  float* data();
  const float* data() const;
};
```

### Requirements
- No leaks and no double frees (the tests run under ASan + LeakSanitizer).
- Self-assignment (copy and move) must leave the buffer unchanged.
- A moved-from buffer is **empty**: `size() == 0`, `data() == nullptr`, and can be safely reassigned or destroyed.
- Moves must not allocate and must be usable by `std::vector<FloatBuffer>` without copying on reallocation. Choose the specifiers that make this true.
- You may change the signatures' specifiers, or merge the two assignment operators into one by-value `operator=`.

### Think about
- Why does `std::vector` care whether your move constructor is `noexcept`?
- How does copy-and-swap give the strong exception guarantee?
- Rule of zero: how would `std::unique_ptr<float[]>` shrink this class? Would copying still work?
- When would you `= delete` the copy operations (a GPU buffer or a sensor handle)?
