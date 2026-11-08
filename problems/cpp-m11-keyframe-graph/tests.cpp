// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  {
    Map map;
    CHECK(map.addKeyframe(1) != nullptr);
    map.addKeyframe(2);
    map.addMapPoint(10, {1, 2, 3});
    map.addMapPoint(11, {4, 5, 6});
    CHECK(map.addObservation(1, 10));
    CHECK(map.addObservation(2, 10) && map.addObservation(2, 11));
    CHECK(map.observersOf(10) == std::vector<int>({1, 2}));

    CHECK(map.removeKeyframe(1));
    CHECK_EQ(Keyframe::live, 1);
    CHECK(map.observersOf(10) == std::vector<int>({2}));
    CHECK_EQ(map.pruneObservations(), std::size_t(1));
    CHECK(map.numKeyframes() == 1 && map.numMapPoints() == 2);
  }
  CHECK(Keyframe::live == 0 && MapPoint::live == 0);

#ifdef HIDDEN
  {
    Map map;
    // duplicates and unknown ids
    map.addKeyframe(1);
    CHECK(map.addKeyframe(1) == nullptr);
    map.addMapPoint(5, {0, 0, 0});
    CHECK(map.addMapPoint(5, {1, 1, 1}) == nullptr);
    CHECK(map.findMapPoint(5) && map.findMapPoint(5)->pos.x == 0);
    CHECK(!map.addObservation(1, 99) && !map.addObservation(42, 5));
    CHECK(map.addObservation(1, 5) && !map.addObservation(1, 5));
    CHECK(map.findMapPoint(5) && map.findMapPoint(5)->observers.size() == 1);
    CHECK(map.observersOf(1234).empty() && !map.removeKeyframe(77) && !map.removeMapPoint(77));

    // caller keeps a keyframe alive after the map drops it
    auto held = map.findKeyframe(1);
    map.removeKeyframe(1);
    CHECK(Keyframe::live == 1 && held && held->points.size() == 1);
    CHECK(map.observersOf(5) == std::vector<int>({1}));  // still alive via `held`
    held.reset();
    CHECK(Keyframe::live == 0 && map.observersOf(5).empty());
    CHECK_EQ(map.pruneObservations(), std::size_t(1));
    CHECK_EQ(map.pruneObservations(), std::size_t(0));

    // removeMapPoint unlinks it from observers so it is destroyed
    map.addKeyframe(2);
    map.addMapPoint(6, {0, 0, 0});
    map.addObservation(2, 5);
    map.addObservation(2, 6);
    CHECK(map.removeMapPoint(5));
    auto kf2 = map.findKeyframe(2);
    CHECK(kf2 && kf2->points.size() == 1 && kf2->points[0]->id == 6);
    CHECK_EQ(MapPoint::live, 1);
  }
  CHECK(Keyframe::live == 0 && MapPoint::live == 0);
  {
    // culling on a bigger graph: point i observed by keyframes [0, i % 5)
    Map map;
    for (int k = 0; k < 4; ++k) map.addKeyframe(k);
    for (int i = 0; i < 1000; ++i) {
      map.addMapPoint(i, {double(i), 0, 0});
      for (int k = 0; k < i % 5; ++k) map.addObservation(k, i);
    }
    map.removeKeyframe(3);  // points with i%5==4 now have 3 live observers
    CHECK_EQ(map.cullMapPoints(2), std::size_t(400));  // i%5 in {0,1}
    CHECK(map.numMapPoints() == 600 && MapPoint::live == 600);
    auto kf0 = map.findKeyframe(0);
    CHECK(kf0 && kf0->points.size() == 600);
    CHECK_EQ(map.cullMapPoints(0), std::size_t(0));
    CHECK(map.observersOf(4) == std::vector<int>({0, 1, 2}));
  }
  CHECK(Keyframe::live == 0 && MapPoint::live == 0);
#endif
  return TEST_SUMMARY();
}
