// The candidate's code is injected before this file (g++ -include).
#include <memory>
#include <type_traits>
#include "common/check.h"

static std::string joined() {
  std::string s;
  for (const auto& e : g_log) s += (s.empty() ? "" : " ") + e;
  return s;
}

int main() {
  g_log.clear();
  { Lidar l("vlp16", 16); }
  CHECK_EQ(joined(), std::string("Sensor(vlp16) +motor Lidar(vlp16) ~Lidar(vlp16) -motor ~Sensor(vlp16)"));

  Lidar lidar("os1", 64, 20);
  lidar.spin(3);
  CHECK_EQ(lidar.frames(), 3);
  CHECK_EQ(lidar.describe(), std::string("os1@20Hz rings=64"));

  g_log.clear();
  { Camera c("cam0", 640, 480); }
  CHECK_EQ(joined(), std::string("Sensor(cam0) +lens +shutter Camera(cam0) ~Camera(cam0) -shutter -lens ~Sensor(cam0)"));

#ifdef HIDDEN
  CHECK(!(std::is_constructible_v<Sensor, std::string, int>));  // protected ctor
  CHECK((std::is_base_of_v<Sensor, Lidar> && std::is_convertible_v<Lidar*, Sensor*>));
  // static type decides which describe() runs (hiding, not overriding)
  const Sensor& base = lidar;
  CHECK_EQ(base.describe(), std::string("os1@20Hz"));
  CHECK_EQ(base.name(), std::string("os1"));
  // defaults and camera frames/description
  Camera cam("cam1", 1280, 720);
  cam.capture();
  cam.capture();
  CHECK(cam.frames() == 2 && cam.rateHz() == 30);
  CHECK_EQ(cam.describe(), std::string("cam1@30Hz 1280x720"));
  lidar.spin(-5);
  CHECK_EQ(lidar.frames(), 3);
  // two objects on the stack: destroyed in reverse order
  g_log.clear();
  { Lidar a("a", 1); Camera b("b", 1, 1); }
  CHECK_EQ(joined(), std::string("Sensor(a) +motor Lidar(a) Sensor(b) +lens +shutter Camera(b) "
                                 "~Camera(b) -shutter -lens ~Sensor(b) ~Lidar(a) -motor ~Sensor(a)"));
  // deleting through a base pointer runs the full chain
  g_log.clear();
  { std::unique_ptr<Sensor> s = std::make_unique<Lidar>("hdl", 32); }
  CHECK_EQ(joined(), std::string("Sensor(hdl) +motor Lidar(hdl) ~Lidar(hdl) -motor ~Sensor(hdl)"));
  // throwing constructor: base + members unwound, derived dtor NOT run
  g_log.clear();
  bool threw = false;
  try { Lidar bad("bad", 0); } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw);
  CHECK_EQ(joined(), std::string("Sensor(bad) +motor -motor ~Sensor(bad)"));
#endif
  return TEST_SUMMARY();
}
