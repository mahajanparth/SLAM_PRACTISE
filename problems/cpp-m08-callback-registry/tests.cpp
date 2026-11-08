// The candidate's code is injected before this file (g++ -include).
#include <memory>
#include <string>
#include "common/check.h"

struct BiasEstimator {
  double sum_az = 0;
  int n = 0;
  void onImu(const ImuSample& s) { sum_az += s.az; ++n; }
};

int main() {
  SensorBus bus;
  std::string order;
  int a = bus.subscribe([&](const ImuSample&) { order += 'a'; });
  int b = bus.subscribe([&](const ImuSample&) { order += 'b'; });
  CHECK(a > 0 && b > 0 && a != b);
  CHECK_EQ(bus.size(), std::size_t(2));
  CHECK_EQ(bus.publish({0.0, 0, 0, 9.81}), std::size_t(2));
  CHECK_EQ(order, std::string("ab"));

  CHECK(bus.unsubscribe(a));
  CHECK(!bus.unsubscribe(a));
  order.clear();
  bus.publish({0.1, 0, 0, 9.81});
  CHECK_EQ(order, std::string("b"));

  BiasEstimator est;
  int m = bus.subscribeMember(&est, &BiasEstimator::onImu);
  bus.publish({0.2, 0, 0, 9.8});
  CHECK(m > 0 && est.n == 1);
  CHECK_NEAR(est.sum_az, 9.8, 1e-12);

#ifdef HIDDEN
  // empty callbacks / null objects rejected
  SensorBus bus2;
  CHECK_EQ(bus2.subscribe(SensorBus::Callback{}), -1);
  CHECK_EQ(bus2.subscribeMember<BiasEstimator>(nullptr, &BiasEstimator::onImu), -1);
  CHECK_EQ(bus2.subscribeMember(&est, static_cast<void (BiasEstimator::*)(const ImuSample&)>(nullptr)), -1);
  CHECK_EQ(bus2.size(), std::size_t(0));
  CHECK_EQ(bus2.publish({}), std::size_t(0));
  // ids never reused
  int i1 = bus2.subscribe([](const ImuSample&) {});
  bus2.unsubscribe(i1);
  int i2 = bus2.subscribe([](const ImuSample&) {});
  CHECK(i2 != i1);

  // re-entrancy: x removes y (later) -> y not called; z removes itself (holds heap state)
  SensorBus bus3;
  std::string log;
  int y = -1, z = -1;
  bus3.subscribe([&](const ImuSample&) { log += 'x'; bus3.unsubscribe(y); });
  y = bus3.subscribe([&](const ImuSample&) { log += 'y'; });
  auto heavy = std::make_shared<std::string>("z");
  z = bus3.subscribe([&, heavy](const ImuSample&) {
    bus3.unsubscribe(z);   // destroys the stored std::function (and its copy of `heavy`)...
    log += *heavy;         // ...so this must still be valid
  });
  heavy.reset();
  CHECK_EQ(bus3.publish({}), std::size_t(2));
  CHECK_EQ(log, std::string("xz"));
  CHECK_EQ(bus3.size(), std::size_t(1));

  // subscribing during publish: new callback runs from the next publish on
  SensorBus bus4;
  int added_calls = 0;
  bool added = false;
  bus4.subscribe([&](const ImuSample&) {
    if (!added) { added = true; bus4.subscribe([&](const ImuSample&) { ++added_calls; }); }
  });
  CHECK_EQ(bus4.publish({}), std::size_t(1));
  CHECK_EQ(added_calls, 0);
  CHECK_EQ(bus4.publish({}), std::size_t(2));
  CHECK_EQ(added_calls, 1);

  // many subscribers, unsubscribe every other one; order kept
  SensorBus bus5;
  std::vector<int> seen, ids;
  for (int i = 0; i < 2000; ++i) ids.push_back(bus5.subscribe([&seen, i](const ImuSample&) { seen.push_back(i); }));
  for (int i = 0; i < 2000; i += 2) bus5.unsubscribe(ids[i]);
  CHECK_EQ(bus5.publish({}), std::size_t(1000));
  bool ordered = seen.size() == 1000;
  for (std::size_t k = 0; ordered && k < seen.size(); ++k) ordered = seen[k] == int(2 * k + 1);
  CHECK(ordered);
#endif
  return TEST_SUMMARY();
}
