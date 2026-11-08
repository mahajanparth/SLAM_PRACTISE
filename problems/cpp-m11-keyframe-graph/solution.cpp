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
    auto [it, inserted] = kfs_.try_emplace(id, nullptr);
    if (!inserted) return nullptr;
    it->second = std::make_shared<Keyframe>(id);
    return it->second;
  }

  std::shared_ptr<MapPoint> addMapPoint(int id, Point3 pos) {
    auto [it, inserted] = mps_.try_emplace(id, nullptr);
    if (!inserted) return nullptr;
    it->second = std::make_shared<MapPoint>(id, pos);
    return it->second;
  }

  bool addObservation(int kf_id, int mp_id) {
    auto kf = findKeyframe(kf_id);
    auto mp = findMapPoint(mp_id);
    if (!kf || !mp) return false;
    if (std::find(kf->points.begin(), kf->points.end(), mp) != kf->points.end()) return false;
    kf->points.push_back(mp);   // strong: keyframe -> point
    mp->observers.push_back(kf);  // weak: point -> keyframe (breaks the cycle)
    return true;
  }

  std::shared_ptr<Keyframe> findKeyframe(int id) const {
    auto it = kfs_.find(id);
    return it == kfs_.end() ? nullptr : it->second;
  }

  std::shared_ptr<MapPoint> findMapPoint(int id) const {
    auto it = mps_.find(id);
    return it == mps_.end() ? nullptr : it->second;
  }

  bool removeKeyframe(int id) {
    // Dropping the map's share. If it was the last owner, ~Keyframe runs here and releases
    // its shares of the map points; the points' weak observers simply expire.
    return kfs_.erase(id) > 0;
  }

  bool removeMapPoint(int id) {
    auto it = mps_.find(id);
    if (it == mps_.end()) return false;
    std::shared_ptr<MapPoint> mp = it->second;
    for (const auto& w : mp->observers) {
      if (auto kf = w.lock()) {  // lock() -> a temporary owner, or null if expired
        auto& pts = kf->points;
        pts.erase(std::remove(pts.begin(), pts.end(), mp), pts.end());
      }
    }
    mps_.erase(it);
    return true;  // `mp` (last owner, unless the caller holds one) dies at scope exit
  }

  std::vector<int> observersOf(int mp_id) const {
    std::vector<int> ids;
    if (auto mp = findMapPoint(mp_id))
      for (const auto& w : mp->observers)
        if (auto kf = w.lock()) ids.push_back(kf->id);
    std::sort(ids.begin(), ids.end());
    return ids;
  }

  std::size_t pruneObservations() {
    std::size_t removed = 0;
    for (auto& [id, mp] : mps_) {
      auto& obs = mp->observers;
      const std::size_t before = obs.size();
      obs.erase(std::remove_if(obs.begin(), obs.end(),
                               [](const std::weak_ptr<Keyframe>& w) { return w.expired(); }),
                obs.end());
      removed += before - obs.size();
    }
    return removed;
  }

  std::size_t cullMapPoints(std::size_t min_obs) {
    std::vector<int> doomed;  // collect first: removeMapPoint mutates mps_
    for (const auto& [id, mp] : mps_) {
      std::size_t alive = 0;
      for (const auto& w : mp->observers) alive += !w.expired();
      if (alive < min_obs) doomed.push_back(id);
    }
    for (int id : doomed) removeMapPoint(id);
    return doomed.size();
  }

  std::size_t numKeyframes() const { return kfs_.size(); }
  std::size_t numMapPoints() const { return mps_.size(); }

 private:
  std::map<int, std::shared_ptr<Keyframe>> kfs_;
  std::map<int, std::shared_ptr<MapPoint>> mps_;
};
