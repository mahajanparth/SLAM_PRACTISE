# Keyframe / MapPoint Graph

A visual SLAM map has **keyframes** that observe **map points**. A keyframe needs its map points to exist (strong reference), and a map point wants to know which keyframes see it (a back-reference that must **not** keep keyframes alive, or the graph leaks through cycles). The types are given:
```cpp
struct MapPoint { int id; Point3 pos; std::vector<std::weak_ptr<Keyframe>> observers; static inline int live; };
struct Keyframe { int id; std::vector<std::shared_ptr<MapPoint>> points;               static inline int live; };
```
(`live` counts constructed-but-not-destroyed objects.) Implement `class Map`:

| Member | Behaviour |
|---|---|
| `std::shared_ptr<Keyframe> addKeyframe(int id)` | Create and own a keyframe. `nullptr` if the id already exists. |
| `std::shared_ptr<MapPoint> addMapPoint(int id, Point3 pos)` | Create and own a map point. `nullptr` if the id already exists. |
| `bool addObservation(int kf_id, int mp_id)` | Keyframe `kf_id` observes point `mp_id`: append the point to `kf->points` and a weak ref to `mp->observers`. `false` (and no change) if either id is not in the map, or the pair is already linked. |
| `std::shared_ptr<Keyframe> findKeyframe(int id) const`, `findMapPoint(int id) const` | Lookup; `nullptr` if absent. |
| `bool removeKeyframe(int id)` | The map drops its keyframe. If nobody else holds it, it's destroyed. Points it observed **stay** in the map. `false` if unknown. |
| `bool removeMapPoint(int id)` | Remove the point from the map **and** from every live observer's `points`. `false` if unknown. |
| `std::vector<int> observersOf(int mp_id) const` | Ids of the **live** keyframes observing the point, ascending. Empty if the point is unknown. |
| `std::size_t pruneObservations()` | Remove expired `weak_ptr`s from every map point's `observers`. Return how many were removed. |
| `std::size_t cullMapPoints(std::size_t min_obs)` | Remove (as in `removeMapPoint`) every point with **fewer than** `min_obs` live observers. Return how many were removed. |
| `numKeyframes()`, `numMapPoints()` | Sizes. |

### Constraints
- When a `Map` is destroyed (and the caller holds no pointers), **every** keyframe and map point must be destroyed (`live == 0`).
- Up to 10⁴ keyframes / points. Lookups are O(log n).

### Think about
- Draw the ownership graph. Which edge would create a cycle if it were a `shared_ptr`?
- Why is `if (!w.expired()) w.lock()->...` a bug in multithreaded code?
- A caller holds a `shared_ptr<Keyframe>` after `removeKeyframe`. What happens to it, and to `observersOf`?
- Real systems (ORB-SLAM) use raw pointers and a `bad` flag instead. What are the trade-offs versus `shared_ptr` (atomic refcounts, memory, determinism of destruction)?
