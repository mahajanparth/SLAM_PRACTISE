#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

Eigen::Isometry3d predictConstantVelocity(const Eigen::Isometry3d& T2, const Eigen::Isometry3d& T1) {
  // TODO
  return Eigen::Isometry3d::Identity();
}

bool isKeyframe(const Eigen::Isometry3d& T_kf, const Eigen::Isometry3d& T_cur, double min_dist, double min_angle) {
  // TODO
  return false;
}

class VoxelHashMap {
 public:
  VoxelHashMap(double voxel_size, int max_points_per_voxel) {
    // TODO
  }
  void addPoints(const Cloud& pts) {
    // TODO
  }
  void removeFarFrom(const Eigen::Vector3d& origin, double max_dist) {
    // TODO
  }
  Cloud pointCloud() const {
    // TODO
    return {};
  }
  size_t numVoxels() const {
    // TODO
    return 0;
  }

 private:
  // TODO: voxel storage
};

struct OdomParams {
  double voxel_size = 0.5;
  int max_points_per_voxel = 20;
  double max_range = 50.0;
  double kf_dist = 1.0;
  double kf_angle = 0.2;
};

class LidarOdometry {
 public:
  using RegisterFn = std::function<Eigen::Isometry3d(const Cloud& scan, const VoxelHashMap& map,
                                                     const Eigen::Isometry3d& init)>;

  LidarOdometry(const OdomParams& params, RegisterFn reg)
      : params_(params), reg_(std::move(reg)), map_(params.voxel_size, params.max_points_per_voxel) {}

  Eigen::Isometry3d process(const Cloud& scan) {
    // TODO
    return Eigen::Isometry3d::Identity();
  }

  const std::vector<Eigen::Isometry3d>& poses() const { return poses_; }
  const VoxelHashMap& map() const { return map_; }

 private:
  OdomParams params_;
  RegisterFn reg_;
  VoxelHashMap map_;
  std::vector<Eigen::Isometry3d> poses_;
  Eigen::Isometry3d last_kf_ = Eigen::Isometry3d::Identity();
};
