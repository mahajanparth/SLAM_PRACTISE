// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  auto parts = split("a;;b", ';');
  CHECK_EQ(parts.size(), 3u);
  if (parts.size() == 3) { CHECK(parts[0] == "a"); CHECK(parts[1].empty()); CHECK(parts[2] == "b"); }
  CHECK(trim("  base_link\t ") == "base_link");
  CHECK(parseDouble(" 10.5 ").value_or(-1) == 10.5);
  CHECK(!parseDouble("10.5hz").has_value());
  CHECK(parseInt("42").value_or(-1) == 42);

  auto c = parseSensor("name=lidar_front; frame=base_link; rate=10; id=3");
  CHECK(c.has_value());
  if (c) {
    CHECK_EQ(c->name, std::string("lidar_front"));
    CHECK_EQ(c->frame, std::string("base_link"));
    CHECK_NEAR(c->rate_hz, 10.0, 1e-12);
    CHECK_EQ(c->id, 3);
  }
  CHECK(!parseSensor("name=imu; frame=imu_link; rate=abc; id=1").has_value());

  std::map<std::string, SensorConfig> rig = {
      {"cam", {"cam", "cam_link", 30.0, 2}}, {"gps", {"gps", "gps_link", 1.0, 4}}, {"imu", {"imu", "imu_link", 200.0, 1}}};
  CHECK(fastSensors(rig, 10.0) == (std::vector<std::string>{"cam", "imu"}));

#ifdef HIDDEN
  CHECK(split("", ';').size() == 1);
  CHECK(split("x;", ';').size() == 2);
  CHECK(trim("   ").empty());
  CHECK(trim("").empty());
  CHECK(parseDouble("-2.5e1").value_or(0) == -25.0);
  CHECK(!parseDouble("").has_value());
  CHECK(!parseInt("3.5").has_value());
  CHECK(!parseInt("").has_value());
  // any key order, extra whitespace, trailing ';'
  auto d = parseSensor("  id = 0 ;rate=  0.5;frame=odom ; name = wheel_odom ; ");
  CHECK(d.has_value() && d->name == "wheel_odom" && d->id == 0 && d->rate_hz == 0.5 && d->frame == "odom");
  // the result must own its strings (input buffer dies)
  std::optional<SensorConfig> owned;
  {
    std::string buf = "name=radar_long_range_front_left; frame=radar_link_front_left; rate=13; id=7";
    owned = parseSensor(buf);
    buf.assign(buf.size(), 'X');
  }
  CHECK(owned && owned->name == "radar_long_range_front_left" && owned->frame == "radar_link_front_left");
  CHECK(!parseSensor("").has_value());
  CHECK(!parseSensor("name=a; frame=b; rate=10").has_value());                 // missing id
  CHECK(!parseSensor("name=a; frame=b; rate=10; id=1; id=2").has_value());     // duplicate
  CHECK(!parseSensor("name=a; frame=b; rate=10; id=1; gain=2").has_value());   // unknown key
  CHECK(!parseSensor("name=a; frame=b; rate 10; id=1").has_value());           // no '='
  CHECK(!parseSensor("name=; frame=b; rate=10; id=1").has_value());            // empty name
  CHECK(!parseSensor("name=a; frame=b; rate=-1; id=1").has_value());           // rate <= 0
  CHECK(!parseSensor("name=a; frame=b; rate=0; id=1").has_value());
  CHECK(!parseSensor("name=a; frame=b; rate=inf; id=1").has_value());          // non-finite
  CHECK(!parseSensor("name=a; frame=b; rate=10; id=-3").has_value());          // negative id
  CHECK(!parseSensor("name=a; frame=b; rate=10; id=1.5").has_value());
  CHECK(fastSensors({}, 1.0).empty());
  CHECK(fastSensors(rig, 1.0).size() == 3);   // inclusive threshold
  CHECK(fastSensors(rig, 1000.0).empty());
#endif
  return TEST_SUMMARY();
}
