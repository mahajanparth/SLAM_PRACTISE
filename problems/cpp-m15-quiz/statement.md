# Predict the Output: Copy/Move, Rule of 0/3/5, `noexcept`

Answer **before** running anything. Interviewers love asking what the compiler generates for you, and when it quietly stops doing so.

Every question assumes:
```cpp
struct Point3 { double x, y, z; };
struct Tracer {
  Tracer() = default;
  Tracer(const Tracer&) { std::cout << "C"; }
  Tracer(Tracer&&) noexcept { std::cout << "M"; }
  Tracer& operator=(const Tracer&) { std::cout << "c"; return *this; }
  Tracer& operator=(Tracer&&) noexcept { std::cout << "m"; return *this; }
};
```
