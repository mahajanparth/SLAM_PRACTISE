# Landmark Observation Counts

The back-end only adds a landmark to the bundle adjustment once it has been seen enough times. You get the front-end's raw observation list, where each entry says "frame `f` saw landmark `l`" (the same landmark can appear many times, even within one frame).

```cpp
struct Observation { int frame_id; int landmark_id; };
```

| Function | Behaviour |
|---|---|
| `std::unordered_map<int, int> countObservations(const std::vector<Observation>& obs)` | `landmark_id → number of observations` |
| `int countOf(const std::unordered_map<int, int>& counts, int id)` | Count for `id`, or 0 if it's absent. `counts` is `const`, so you must not insert |
| `int mostObserved(const std::vector<Observation>& obs)` | The landmark with the most observations. Ties go to the **smallest id**. Returns `-1` for empty input |
| `std::vector<int> stableLandmarks(const std::vector<Observation>& obs, int min_count)` | Ids with `count ≥ min_count`, **ascending** |

Ids can be any `int`, including negative values.

### Constraints
- Up to 2·10⁵ observations. Expected average O(n) for the first three, O(n log m) for `stableLandmarks` (m = distinct ids).

### Think about
- Why doesn't `counts[id]` compile on a `const std::unordered_map&`? What silent bug does it cause on a non-const map?
- `std::map` vs `std::unordered_map`: complexity, memory per node, iteration order, and worst case.
- What does `reserve(n)` do for an `unordered_map`? When does it rehash?
- How would you do the same counting with a sorted `std::vector<int>` and no hashing at all?
