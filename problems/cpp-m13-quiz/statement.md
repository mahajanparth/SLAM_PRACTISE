# Predict the Output: Construction Order, Access, Name Hiding

Answer **before** running anything. Sensor-driver class hierarchies are the classic setting for these questions.

Every question assumes:
```cpp
struct Tracer {
  const char* n;
  explicit Tracer(const char* s) : n(s) { std::cout << "+" << n << " "; }
  ~Tracer() { std::cout << "-" << n << " "; }
};
```
