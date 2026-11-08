// The candidate's code is injected before this file (g++ -include).
#include <limits>
#include "common/check.h"

int main() {
  std::vector<Msg> msgs = {
      ImuMsg{0.00, 0.5, 0.0},
      LidarMsg{0.05, {1.0f, 0.0f, 2.5f, std::numeric_limits<float>::infinity()}},
      ImuMsg{0.10, 1.0, 0.0},
      OdomMsg{0.12, 1.0, 2.0, 0.1},
      ImuMsg{0.20, 0.0, 0.0},
  };
  CHECK_NEAR(stampOf(msgs[1]), 0.05, 1e-12);
  CHECK_NEAR(stampOf(msgs[3]), 0.12, 1e-12);

  auto imus = extract<ImuMsg>(msgs);
  CHECK_EQ(imus.size(), 3u);
  if (imus.size() == 3) CHECK_NEAR(imus[1].gyro_z, 1.0, 1e-12);

  Summary s = process(msgs);
  CHECK(s.lidar == 1 && s.imu == 3 && s.odom == 1);
  CHECK_EQ(s.valid_ranges, 2u);
  CHECK_NEAR(s.yaw_from_gyro, 0.5 * 0.1 + 1.0 * 0.1, 1e-12);
  CHECK(s.last_odom.has_value() && s.last_odom->x == 1.0);

  std::vector<Msg> shuffled = {OdomMsg{3.0, 0, 0, 0}, ImuMsg{1.0, 0, 0}, LidarMsg{2.0, {}}};
  sortByStamp(shuffled);
  CHECK(std::holds_alternative<ImuMsg>(shuffled[0]) && std::holds_alternative<OdomMsg>(shuffled[2]));

#ifdef HIDDEN
  // empty stream
  Summary e = process({});
  CHECK(e.lidar == 0 && e.imu == 0 && e.odom == 0 && e.valid_ranges == 0 && e.yaw_from_gyro == 0.0 && !e.last_odom);
  CHECK(extract<LidarMsg>({}).empty());
  CHECK(!latestOdomBefore({}, 1.0).has_value());
  // single IMU: no integration; NaN / negative ranges invalid
  Summary one = process({ImuMsg{1.0, 5.0, 0.0}, LidarMsg{1.1, {std::nanf(""), -1.0f, 0.1f}}});
  CHECK(one.yaw_from_gyro == 0.0 && one.valid_ranges == 1);
  // stable sort for ties
  std::vector<Msg> ties = {OdomMsg{1.0, 1, 0, 0}, ImuMsg{0.5, 0, 0}, OdomMsg{1.0, 2, 0, 0}, OdomMsg{1.0, 3, 0, 0}};
  sortByStamp(ties);
  auto od = extract<OdomMsg>(ties);
  CHECK(std::holds_alternative<ImuMsg>(ties[0]));
  CHECK(od.size() == 3 && od[0].x == 1 && od[1].x == 2 && od[2].x == 3);
  // last_odom: last one in stream among equal max stamps
  Summary st = process(ties);
  CHECK(st.last_odom && st.last_odom->x == 3);
  // latestOdomBefore on unsorted input, boundary inclusive, first among ties
  std::vector<Msg> un = {OdomMsg{5.0, 5, 0, 0}, OdomMsg{2.0, 2, 0, 0}, ImuMsg{4.0, 0, 0}, OdomMsg{4.0, 4, 0, 0}, OdomMsg{4.0, 44, 0, 0}};
  auto lo = latestOdomBefore(un, 4.5);
  CHECK(lo && lo->x == 4);
  auto lb = latestOdomBefore(un, 2.0);
  CHECK(lb && lb->x == 2);
  CHECK(!latestOdomBefore(un, 1.9).has_value());
  // large stream: integration with constant rate
  std::vector<Msg> big;
  for (int i = 0; i <= 1000; ++i) {
    big.push_back(ImuMsg{i * 0.001, 0.2, 0.0});
    if (i % 10 == 0) big.push_back(LidarMsg{i * 0.001, std::vector<float>(36, 1.0f)});
  }
  Summary b = process(big);
  CHECK_NEAR(b.yaw_from_gyro, 0.2, 1e-9);
  CHECK(b.imu == 1001 && b.lidar == 101 && b.valid_ranges == 101u * 36u);
  CHECK(extract<LidarMsg>(big).size() == 101);
#endif
  return TEST_SUMMARY();
}
