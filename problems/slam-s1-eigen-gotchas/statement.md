# Bug Hunt: Eigen Gotchas

An interviewer hands you this snippet and asks *"What's wrong with it?"*. Each question below targets one line.

```cpp
struct Pose { Eigen::Vector4d q; Eigen::Vector3d t; };
std::vector<Pose> poses;                                   // (a)
Eigen::MatrixXd A = Eigen::MatrixXd::Random(3, 3);
A = A.transpose();                                         // (b)
auto C = A * A;                                            // (c) then A changes, C is used later
Eigen::Matrix3d M; M.block<2,2>(0,0) = Eigen::Matrix2d::Identity(); // (d) rest?
Eigen::Vector3d v = Eigen::Vector3d::Zero(); v(3) = 1;    // (e)
```
