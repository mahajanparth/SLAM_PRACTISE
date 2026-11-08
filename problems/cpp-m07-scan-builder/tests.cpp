// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  ScanBuilder b;
  b.stamp(12.5).frame("velodyne").addRange(1.0).addRanges({2.0, 3.0});
  CHECK(b.ranges().size() == 3);

  LidarScan s1 = b.build();  // lvalue -> copy, builder intact
  CHECK_NEAR(s1.stamp, 12.5, 1e-12);
  CHECK(s1.frame_id == "velodyne");
  CHECK(s1.ranges.size() == 3);
  CHECK(b.ranges().size() == 3);

  const double* buf = b.ranges().data();
  LidarScan s2 = std::move(b).build();  // rvalue -> move
  CHECK(s2.ranges.size() == 3);
  CHECK(s2.ranges.data() == buf);  // buffer was stolen, not copied

  std::vector<double> r = {0.0, 5.0, 250.0};
  std::vector<double> c = clampRanges(r);  // defaults [0.1, 100]
  CHECK(c.size() == 3);
  if (c.size() == 3) { CHECK_NEAR(c[0], 0.1, 1e-12); CHECK_NEAR(c[2], 100.0, 1e-12); }

#ifdef HIDDEN
  CHECK_NEAR(r[2], 250.0, 1e-12);  // lvalue argument unchanged
  // move all the way through the sink parameter
  std::vector<double> big(5000, 500.0);
  const double* bigBuf = big.data();
  std::vector<double> clamped = clampRanges(std::move(big), 1.0, 60.0);
  CHECK(clamped.data() == bigBuf);
  CHECK_NEAR(clamped[4999], 60.0, 1e-12);
  // custom bounds, empty input
  std::vector<double> lo = clampRanges({-1.0, 0.5}, 0.0, 1.0);
  CHECK(lo.size() == 2 && lo[0] == 0.0 && lo[1] == 0.5);
  CHECK(clampRanges({}).empty());
  // build twice from the same lvalue builder
  ScanBuilder b2;
  b2.frame(std::string("os1")).addRange(7.0);
  LidarScan a = b2.build(), a2 = b2.build();
  CHECK(a.ranges == a2.ranges && a2.frame_id == "os1");
  // temporary builder chain: frame survives
  LidarScan t = ScanBuilder().stamp(3.0).frame("ouster").addRanges({1, 2, 3, 4}).build();
  CHECK(t.ranges.size() == 4 && t.frame_id == "ouster" && t.stamp == 3.0);
  // appending a builder's own ranges to itself
  ScanBuilder b3;
  b3.addRanges({1.0, 2.0});
  b3.addRanges(b3.ranges());
  CHECK(b3.ranges().size() == 4);
  if (b3.ranges().size() == 4) CHECK_NEAR(b3.ranges()[3], 2.0, 1e-12);
  // a long string moved in keeps its heap buffer
  std::string longId(64, 'x');
  const char* sbuf = longId.data();
  ScanBuilder b4;
  b4.frame(std::move(longId));
  LidarScan s4 = std::move(b4).build();
  CHECK(s4.frame_id.data() == sbuf);
#endif
  return TEST_SUMMARY();
}
