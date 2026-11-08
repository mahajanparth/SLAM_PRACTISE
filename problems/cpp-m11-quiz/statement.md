# Predict the Output: `unique_ptr`, `shared_ptr`, `weak_ptr`, Cycles

Answer **before** running anything. Ownership questions are the heart of most C++ robotics interviews: who owns the keyframes, who merely observes the map points, and why the map never frees memory.

Every question assumes:
```cpp
struct Point3 { double x, y, z; };
struct Keyframe { int id; std::vector<Point3> cloud; };
```
