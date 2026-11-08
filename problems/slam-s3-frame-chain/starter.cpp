#include <Eigen/Dense>
#include <Eigen/Geometry>

Eigen::Vector3d lidarPointInCamera(const Eigen::Isometry3d& T_world_robot,
                                   const Eigen::Isometry3d& T_robot_lidar,
                                   const Eigen::Isometry3d& T_world_camera,
                                   const Eigen::Vector3d& p_lidar) {
  // TODO
  return p_lidar;
}

Eigen::Isometry3d relativePose(const Eigen::Isometry3d& T_w_a, const Eigen::Isometry3d& T_w_b) {
  // TODO: return T_a_b
  return Eigen::Isometry3d::Identity();
}

Eigen::Isometry3d predictPose(const Eigen::Isometry3d& T_w_a, const Eigen::Isometry3d& T_a_b) {
  // TODO: return T_w_b
  return Eigen::Isometry3d::Identity();
}
