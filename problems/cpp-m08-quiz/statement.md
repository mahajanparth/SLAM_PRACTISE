# Predict the Output: Function Pointers, `std::function`, Member Pointers

Answer **before** running anything. Callbacks are everywhere in robotics stacks: sensor drivers, ROS subscriptions, state machines. Interviewers like to check that you know what each kind of callable costs and how it can fail.

Every question assumes:
```cpp
struct Point3 { double x, y, z; };
```
