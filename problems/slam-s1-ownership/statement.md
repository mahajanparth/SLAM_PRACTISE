# Design: Smart-Pointer Ownership in a SLAM Map

The interviewer asks: *"Design classes `Map`, `Keyframe` and `MapPoint`. Keyframes observe map points, and map points know which keyframes see them. Choose pointer types, write the members and `addObservation`."*

These questions walk through the trade-offs a strong answer discusses.

```cpp
struct MapPoint; struct Keyframe;
struct Map {
  std::unordered_map<int, std::shared_ptr<Keyframe>> keyframes;
  std::unordered_map<int, std::shared_ptr<MapPoint>> points;
};
```
