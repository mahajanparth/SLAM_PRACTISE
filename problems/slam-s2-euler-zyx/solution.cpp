#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

Eigen::Matrix3d eulerZYXToRot(double yaw, double pitch, double roll) {
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cr = std::cos(roll), sr = std::sin(roll);
  Eigen::Matrix3d R;  // Rz(yaw) * Ry(pitch) * Rx(roll), multiplied out
  R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
       sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
       -sp,     cp * sr,                cp * cr;
  return R;
}

Eigen::Vector3d rotToEulerZYX(const Eigen::Matrix3d& R) {
  const double cp = std::hypot(R(0, 0), R(1, 0));  // |cos(pitch)| >= 0
  const double pitch = std::atan2(-R(2, 0), cp);   // in [-pi/2, pi/2], no asin clamping needed
  if (cp < 1e-9) {
    // Gimbal lock: the first column and last row vanish and only yaw -/+ roll is observable.
    // Setting roll := 0, both pitch = +90 and -90 give R(0,1) = -sin(yaw), R(1,1) = cos(yaw).
    return Eigen::Vector3d(std::atan2(-R(0, 1), R(1, 1)), pitch, 0.0);
  }
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  return Eigen::Vector3d(yaw, pitch, roll);
}
