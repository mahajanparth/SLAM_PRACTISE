#include <algorithm>
#include <cstddef>
#include <map>
#include <memory>
#include <vector>

struct Point3 { double x, y, z; };
struct Keyframe;

struct MapPoint {
  int id;
  Point3 pos;
  std::vector<std::weak_ptr<Keyframe>> observers;  // back-edges: do NOT own
  static inline int live = 0;
  MapPoint(int id_, Point3 p) : id(id_), pos(p) { ++live; }
  ~MapPoint() { --live; }
};

struct Keyframe {
  int id;
  std::vector<std::shared_ptr<MapPoint>> points;  // a keyframe keeps its points alive
  static inline int live = 0;
  explicit Keyframe(int id_) : id(id_) { ++live; }
  ~Keyframe() { --live; }
};

class Map {
 public:
  std::shared_ptr<Keyframe> addKeyframe(int id) {
    // TODO
    return nullptr;
  }
  std::shared_ptr<MapPoint> addMapPoint(int id, Point3 pos) {
    // TODO
    return nullptr;
  }
  bool addObservation(int kf_id, int mp_id) {
    // TODO
    return false;
  }
  std::shared_ptr<Keyframe> findKeyframe(int id) const {
    // TODO
    return nullptr;
  }
  std::shared_ptr<MapPoint> findMapPoint(int id) const {
    // TODO
    return nullptr;
  }
  bool removeKeyframe(int id) {
    // TODO
    return false;
  }
  bool removeMapPoint(int id) {
    // TODO
    return false;
  }
  std::vector<int> observersOf(int mp_id) const {
    // TODO
    return {};
  }
  std::size_t pruneObservations() {
    // TODO
    return 0;
  }
  std::size_t cullMapPoints(std::size_t min_obs) {
    // TODO
    return 0;
  }
  std::size_t numKeyframes() const { return 0; /* TODO */ }
  std::size_t numMapPoints() const { return 0; /* TODO */ }

 private:
  std::map<int, std::shared_ptr<Keyframe>> kfs_;
  std::map<int, std::shared_ptr<MapPoint>> mps_;
};
