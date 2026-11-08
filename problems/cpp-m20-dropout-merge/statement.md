# Sensor Dropouts

The health monitor logs a time window each time a sensor stops publishing. The logs are unordered, overlapping, and
sometimes split by tiny gaps. Clean them up and answer availability questions for the mission report.

```cpp
struct Interval { double start, end; };   // half-open [start, end), start <= end
```

| Function | Behaviour |
|---|---|
| `std::vector<Interval> mergeDropouts(std::vector<Interval> v, double max_gap)` | Sorted by `start`, non-overlapping. Two windows merge if they overlap **or** the gap between them is `<= max_gap` (`max_gap >= 0`). Touching windows (`a.end == b.start`) always merge. Zero-length windows are kept unless they merge into a neighbour |
| `bool isDown(const std::vector<Interval>& merged, double t)` | `t` lies in some `[start, end)`. **O(log n)** |
| `double uptime(const std::vector<Interval>& merged, double t0, double t1)` | Length of `[t0, t1)` **not** covered by `merged`; `0` if `t1 <= t0` |
| `std::vector<Interval> bothDown(const std::vector<Interval>& a, const std::vector<Interval>& b)` | Windows when **both** sensors were down (`a`, `b` already merged). Only non-empty overlaps, sorted. **O(n + m)** |

### Example
```
mergeDropouts({{5,6},{1,3},{2,4},{4.05,4.5}}, 0.1)  ->  {{1,4.5},{5,6}}
```

### Think about
- Clarifying questions: closed or half-open? Can `end < start` occur in the raw log (clock jumps)? How should a gap be handled?
- Why sort a **copy** (the parameter is by value) rather than the caller's data?
- How do you keep the merged list up to date as windows stream in live (`std::map` keyed by start)?
- What is the complexity of `uptime` for Q queries? Can prefix sums make it O(log n) per query?
