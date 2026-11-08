#include <Eigen/Dense>
#include <Eigen/Geometry>

// T_camera_lidar = T_camera_world * T_world_robot * T_robot_lidar (adjacent subscripts cancel).
// For many points, compute T_camera_lidar once and apply it to the whole cloud.
Eigen::Vector3d lidarPointInCamera(const Eigen::Isometry3d& T_world_robot,
                                   const Eigen::Isometry3d& T_robot_lidar,
                                   const Eigen::Isometry3d& T_world_camera,
                                   const Eigen::Vector3d& p_lidar) {
  const Eigen::Isometry3d T_camera_lidar = T_world_camera.inverse() * T_world_robot * T_robot_lidar;
  return T_camera_lidar * p_lidar;
}

// T_a_b = T_a_w * T_w_b
Eigen::Isometry3d relativePose(const Eigen::Isometry3d& T_w_a, const Eigen::Isometry3d& T_w_b) {
  return T_w_a.inverse() * T_w_b;
}

// T_w_b = T_w_a * T_a_b
Eigen::Isometry3d predictPose(const Eigen::Isometry3d& T_w_a, const Eigen::Isometry3d& T_a_b) {
  return T_w_a * T_a_b;
}
