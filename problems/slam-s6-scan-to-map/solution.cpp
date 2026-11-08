#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

// Constant body-frame velocity: replay the last relative motion Δ = T2⁻¹ T1 on top of T1.
Eigen::Isometry3d predictConstantVelocity(const Eigen::Isometry3d& T2, const Eigen::Isometry3d& T1) {
  return T1 * (T2.inverse() * T1);
}

bool isKeyframe(const Eigen::Isometry3d& T_kf, const Eigen::Isometry3d& T_cur, double min_dist, double min_angle) {
  const Eigen::Isometry3d d = T_kf.inverse() * T_cur;  // motion since the last keyframe
  return d.translation().norm() >= min_dist || Eigen::AngleAxisd(d.linear()).angle() >= min_angle;
}

class VoxelHashMap {
 public:
  VoxelHashMap(double voxel_size, int max_points_per_voxel)
      : voxel_size_(voxel_size), max_pts_(max_points_per_voxel) {}

  void addPoints(const Cloud& pts) {
    for (const auto& p : pts) {
      auto& bucket = grid_[key(p)];
      if (static_cast<int>(bucket.size()) < max_pts_) bucket.push_back(p);  // bounded density
    }
  }

  void removeFarFrom(const Eigen::Vector3d& origin, double max_dist) {
    const double r2 = max_dist * max_dist;
    for (auto it = grid_.begin(); it != grid_.end();) {
      if (it->second.empty() || (it->second.front() - origin).squaredNorm() > r2)
        it = grid_.erase(it);  // erase returns the next valid iterator
      else
        ++it;
    }
  }

  Cloud pointCloud() const {
    Cloud out;
    for (const auto& kv : grid_) out.insert(out.end(), kv.second.begin(), kv.second.end());
    return out;
  }

  size_t numVoxels() const { return grid_.size(); }

 private:
  struct Hash {
    std::size_t operator()(const Eigen::Vector3i& v) const {
      return static_cast<std::size_t>(v.x()) * 73856093u ^ static_cast<std::size_t>(v.y()) * 19349663u ^
             static_cast<std::size_t>(v.z()) * 83492791u;
    }
  };
  struct Eq {
    bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const { return a == b; }
  };
  Eigen::Vector3i key(const Eigen::Vector3d& p) const {
    return (p / voxel_size_).array().floor().cast<int>();  // floor: correct for negative coords
  }

  double voxel_size_;
  int max_pts_;
  std::unordered_map<Eigen::Vector3i, std::vector<Eigen::Vector3d>, Hash, Eq> grid_;
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
    if (poses_.empty()) {  // bootstrap: first scan defines the world frame
      poses_.push_back(Eigen::Isometry3d::Identity());
      updateMap(scan, poses_.back());
      return poses_.back();
    }
    const size_t k = poses_.size();
    const Eigen::Isometry3d init = k >= 2 ? predictConstantVelocity(poses_[k - 2], poses_[k - 1]) : poses_[k - 1];
    const Eigen::Isometry3d T = reg_(scan, map_, init);
    poses_.push_back(T);
    if (isKeyframe(last_kf_, T, params_.kf_dist, params_.kf_angle)) updateMap(scan, T);
    return T;
  }

  const std::vector<Eigen::Isometry3d>& poses() const { return poses_; }
  const VoxelHashMap& map() const { return map_; }

 private:
  void updateMap(const Cloud& scan, const Eigen::Isometry3d& T) {
    Cloud world;
    world.reserve(scan.size());
    for (const auto& p : scan) world.push_back(T * p);
    map_.addPoints(world);
    map_.removeFarFrom(T.translation(), params_.max_range);  // sliding local map
    last_kf_ = T;
  }

  OdomParams params_;
  RegisterFn reg_;
  VoxelHashMap map_;
  std::vector<Eigen::Isometry3d> poses_;
  Eigen::Isometry3d last_kf_ = Eigen::Isometry3d::Identity();
};
