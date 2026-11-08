# Predict the Output: Polymorphism

Answer **before** running anything. Interviewers use these to check that you know what the vtable actually does.

Unless stated otherwise, every question assumes:
```cpp
struct Sensor {
  virtual ~Sensor() = default;
  virtual std::string name() const { return "sensor"; }
};
struct Lidar : Sensor {
  std::string name() const override { return "lidar"; }
};
```
