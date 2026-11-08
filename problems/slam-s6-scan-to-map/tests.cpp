// The candidate's code is injected before this file (g++ -include).
#include <algorithm>
#include <vector>
#include "common/check.h"
#include "common/cloud_utils.h"

static Eigen::Isometry3d pose(double yaw, double x, double y) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, 0);
  return T;
}
static Eigen::Matrix4d M(const Eigen::Isometry3d& T) { return T.matrix(); }  // by value for CHECK_MAT_NEAR

int main() {
  // Constant velocity: moving forward 1 m and turning 0.1 rad per step, in the BODY frame.
  {
    const Eigen::Isometry3d step = pose(0.1, 1.0, 0.0);
    const Eigen::Isometry3d T2 = pose(0.3, 2, -1), T1 = T2 * step;
    CHECK_MAT_NEAR(M(predictConstantVelocity(T2, T1)), M(T1 * step), 1e-12);
  }
  // Keyframe thresholds (translation OR rotation).
  {
    const Eigen::Isometry3d K = pose(1.0, 5, 5);
    CHECK(!isKeyframe(K, K * pose(0.05, 0.5, 0), 1.0, 0.2));
    CHECK(isKeyframe(K, K * pose(0.0, 0.8, 0.8), 1.0, 0.2));
    CHECK(isKeyframe(K, K * pose(0.3, 0.0, 0.0), 1.0, 0.2));
  }
  // Voxel map: capacity per voxel, removal by distance.
  {
    VoxelHashMap m(1.0, 2);
    m.addPoints({{0.1, 0.1, 0.1}, {0.2, 0.2, 0.2}, {0.3, 0.3, 0.3}, {5.5, 0, 0}});
    CHECK_EQ(m.numVoxels(), size_t(2));
    CHECK_EQ(m.pointCloud().size(), size_t(3));  // third point in a full voxel is dropped
    m.removeFarFrom({0, 0, 0}, 3.0);
    CHECK_EQ(m.numVoxels(), size_t(1));
  }
  // Orchestration with a fake registrator that records init and returns ground truth.
  {
    std::vector<Eigen::Isometry3d> truth = {Eigen::Isometry3d::Identity(), pose(0.0, 0.4, 0), pose(0.0, 0.8, 0),
                                            pose(0.0, 1.2, 0)};
    std::vector<Eigen::Isometry3d> inits;
    size_t k = 0;
    OdomParams prm;
    prm.voxel_size = 0.25;
    prm.kf_dist = 1.0;
    LidarOdometry odo(prm, [&](const Cloud&, const VoxelHashMap&, const Eigen::Isometry3d& init) {
      inits.push_back(init);
      return truth[std::min(++k, truth.size() - 1)];
    });
    const Cloud scan = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    CHECK_MAT_NEAR(M(odo.process(scan)), Eigen::Matrix4d::Identity(), 1e-12);
    CHECK_EQ(odo.map().pointCloud().size(), size_t(3));
    for (int i = 0; i < 3; ++i) odo.process(scan);
    CHECK_EQ(odo.poses().size(), size_t(4));
    CHECK_EQ(inits.size(), size_t(3));
    if (inits.size() == 3) {
      CHECK_MAT_NEAR(M(inits[0]), M(truth[0]), 1e-12);  // only one pose -> last pose
      CHECK_MAT_NEAR(M(inits[2]), M(pose(0.0, 1.2, 0)), 1e-12);  // constant velocity
    }
    // 0.4 and 0.8 m are below kf_dist; 1.2 m triggers a keyframe -> 3 more points.
    CHECK_EQ(odo.map().pointCloud().size(), size_t(6));
  }

#ifdef HIDDEN
  // Constant velocity from identical poses = no motion; full 3D motion.
  {
    const Eigen::Isometry3d A = cu::randomPose(M_PI, 3.0);
    CHECK_MAT_NEAR(M(predictConstantVelocity(A, A)), M(A), 1e-12);
    const Eigen::Isometry3d d = cu::randomPose(0.2, 0.5);
    CHECK_MAT_NEAR(M(predictConstantVelocity(A, A * d)), M(A * d * d), 1e-12);
  }
  // Keyframe test is in the keyframe's frame: large world-frame offset of both poses doesn't matter.
  {
    const Eigen::Isometry3d K = cu::randomPose(M_PI, 100.0);
    CHECK(!isKeyframe(K, K, 0.5, 0.1));
    CHECK(isKeyframe(K, K * pose(0.0, 0.3, 0.45), 0.5, 0.1));  // |t| = 0.54 split over x and y
    CHECK(!isKeyframe(K, K * pose(-0.09, 0.0, 0.3), 0.5, 0.1));
    CHECK(isKeyframe(K, K * pose(-0.15, 0.0, 0.0), 0.5, 0.1));  // negative rotation
  }
  // Negative coordinates: floor, not truncation (-0.4 and +0.4 are different voxels).
  {
    VoxelHashMap m(1.0, 5);
    m.addPoints({{-0.4, 0.2, 0.2}, {0.4, 0.2, 0.2}, {-0.6, 0.1, 0.9}});
    CHECK_EQ(m.numVoxels(), size_t(2));
    m.removeFarFrom({100, 0, 0}, 1.0);  // everything far
    CHECK_EQ(m.numVoxels(), size_t(0));
    CHECK(m.pointCloud().empty());
  }
  // Removal uses the FIRST point of each voxel; many voxels erased while iterating.
  {
    VoxelHashMap m(0.5, 3);
    m.addPoints(cu::uniformCube(2000, 10.0));
    const size_t before = m.numVoxels();
    m.removeFarFrom({0, 0, 0}, 5.0);
    const Cloud left = m.pointCloud();
    CHECK(m.numVoxels() < before && m.numVoxels() > 0);
    // voxel of side 0.5 → any point is within diag 0.87 of its voxel's first point
    CHECK(std::all_of(left.begin(), left.end(), [](const Eigen::Vector3d& p) { return p.norm() <= 5.0 + 0.87; }));
  }
  // Sliding map: a far keyframe evicts the old map.
  {
    OdomParams prm;
    prm.max_range = 10.0;
    LidarOdometry odo(prm, [](const Cloud&, const VoxelHashMap&, const Eigen::Isometry3d&) {
      return pose(0.0, 50.0, 0.0);
    });
    const Cloud scan = {{1, 0, 0}, {2, 0, 0}};
    odo.process(scan);
    odo.process(scan);
    const Cloud mp = odo.map().pointCloud();
    CHECK_EQ(mp.size(), size_t(2));
    CHECK(std::all_of(mp.begin(), mp.end(), [](const Eigen::Vector3d& p) { return p.x() > 50.0; }));
  }
#endif
  return TEST_SUMMARY();
}
