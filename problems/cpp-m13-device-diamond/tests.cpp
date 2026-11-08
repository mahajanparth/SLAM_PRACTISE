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
  { SmartLidar s(7, 12.0, "10.0.0.7"); }
  CHECK_EQ(joined(), std::string("Device(7) Powered(7) Networked(7) SmartLidar(7) "
                                 "~SmartLidar(7) ~Networked(7) ~Powered(7) ~Device(7)"));

  SmartLidar s(3, 9.5, "192.168.1.201");
  s.powerOn();
  s.connect();
  CHECK_EQ(s.totalBoots(), 2);
  CHECK_EQ(s.deviceId(), 3);
  CHECK(s.Powered::bootCount() == 2 && s.Networked::bootCount() == 2);  // one shared counter

  g_log.clear();
  { Powered p(4, 5.0); }
  CHECK_EQ(joined(), std::string("Device(4) Powered(4) ~Powered(4) ~Device(4)"));

#ifdef HIDDEN
  // a single Device subobject: both paths lead to the same address
  const Device* viaP = static_cast<const Device*>(static_cast<const Powered*>(&s));
  const Device* viaN = static_cast<const Device*>(static_cast<const Networked*>(&s));
  CHECK(viaP == viaN);
  CHECK(s.watts() == 9.5 && s.ip() == "192.168.1.201" && s.isOn());
  // powerOn is idempotent; connect is not
  s.powerOn();
  s.connect();
  CHECK_EQ(s.totalBoots(), 3);
  // standalone Networked boots its own Device
  g_log.clear();
  {
    Networked n(9, "10.0.0.9");
    n.connect();
    n.connect();
    CHECK(n.bootCount() == 2 && n.id() == 9);
  }
  CHECK_EQ(joined(), std::string("Device(9) Networked(9) ~Networked(9) ~Device(9)"));
  // deletion through either base pointer runs the full chain exactly once
  g_log.clear();
  { std::unique_ptr<Powered> p = std::make_unique<SmartLidar>(11, 1.0, "a"); }
  CHECK_EQ(joined(), std::string("Device(11) Powered(11) Networked(11) SmartLidar(11) "
                                 "~SmartLidar(11) ~Networked(11) ~Powered(11) ~Device(11)"));
  g_log.clear();
  { std::unique_ptr<Networked> n = std::make_unique<SmartLidar>(12, 1.0, "b"); }
  CHECK_EQ(g_log.size(), std::size_t(8));
  // two lidars: independent devices, reverse destruction order
  g_log.clear();
  {
    SmartLidar a(1, 1, "x"), b(2, 2, "y");
    a.connect();
    CHECK(a.totalBoots() == 1 && b.totalBoots() == 0 && !b.isOn());
  }
  CHECK(g_log.size() == 16 && g_log[8] == "~SmartLidar(2)" && g_log[15] == "~Device(1)");
  CHECK((std::is_base_of_v<Device, SmartLidar> && std::has_virtual_destructor_v<Device>));
#endif
  return TEST_SUMMARY();
}
