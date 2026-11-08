// Toolchain sanity check: ./run.sh scratch/s00_hello.cpp
#include "common/check.h"
#include "common/cloud_utils.h"

int main() {
  // Rotation round-trip: R -> quaternion -> R
  Eigen::Matrix3d R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  Eigen::Quaterniond q(R);
  CHECK_MAT_NEAR(q.toRotationMatrix(), R, 1e-12);
  CHECK_NEAR(R.determinant(), 1.0, 1e-12);

  // Cloud transform round-trip
  auto cloud = cu::uniformCube(1000);
  auto T = cu::randomPose();
  auto back = cu::transform(cu::transform(cloud, T), T.inverse());
  CHECK_MAT_NEAR(cu::toMatrix(back), cu::toMatrix(cloud), 1e-9);
  CHECK_EQ(cloud.size(), size_t{1000});

  return TEST_SUMMARY();
}
