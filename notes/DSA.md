# DSA · C++ Data Structures for Robotics

## Core ideas
### Ring buffer (IMU history, fixed memory, O(1))
```cpp
buf_[head_] = v; head_ = (head_ + 1) % N; if (size_ < N) ++size_;   // push, overwrite oldest
const T& at(i) { return buf_[(head_ + N - size_ + i) % N]; }        // i = 0 oldest
```
Power-of-two `N` gives `& (N-1)` instead of `%`. A lock-free SPSC version uses `std::atomic<size_t> head, tail`
(producer stores with `release`, consumer loads with `acquire`).

### Thread-safe queue (sensor thread → estimator thread)
```cpp
std::optional<T> pop() {
  std::unique_lock lk(m_);
  cv_.wait(lk, [&]{ return shutdown_ || !q_.empty(); });  // predicate: spurious wakeups
  if (q_.empty()) return std::nullopt;                      // shut down and drained
  T v = std::move(q_.front()); q_.pop_front(); return v;
}
void shutdown() { { std::lock_guard lk(m_); shutdown_ = true; } cv_.notify_all(); }
```

### Nearest timestamp (sorted 100 Hz poses)
```cpp
auto it = std::lower_bound(v.begin(), v.end(), t, [](const S& s, double t){ return s.t < t; });
// candidates: it and it-1; guard begin()/end(); reject if |dt| > max_gap
p = (1-a)*p0 + a*p1;   q = q0.slerp(a, q1);   // a = (t - t0)/(t1 - t0)
```

### LRU cache
```cpp
std::list<std::pair<K,V>> items;                       // front = most recent
std::unordered_map<K, decltype(items)::iterator> idx;
items.splice(items.begin(), items, it);                // O(1) move-to-front, no invalidation
```

### Streaming median: two heaps
`lo` max-heap (lower half), `hi` min-heap (upper half). Keep `|lo| == |hi|` or `|lo| == |hi|+1`.
Median is `lo.top()` (odd count) or `(lo.top()+hi.top())/2` (even count).

### Grid connected components
Row-major scan; BFS with an explicit `std::queue`; **label on push**; 4- or 8-neighbour offsets; bounds-check first.

## What happens in memory / cost table
| Structure | Op | Cost | Memory notes |
|---|---|---|---|
| `std::array` ring | push / index | O(1) | contiguous, no allocation, cache friendly |
| `std::deque` queue | push/pop ends | O(1) amortized | chunked blocks; pointers to elements stable on end insert |
| `std::vector` + `lower_bound` | query | O(log n) | contiguous binary search |
| `std::list` | splice / erase | O(1) | one heap node per element, poor locality |
| `unordered_map` | find | O(1) avg | node-based: rehash invalidates **iterators**, not pointers |
| `priority_queue` | push/pop | O(log n) | binary heap inside a vector |
| `std::multiset` | insert/erase | O(log n) | red-black tree; used for sliding-window median |
| BFS on R×C grid | total | O(RC) | queue ≤ O(RC); recursion would risk stack overflow |

## Pitfalls interviewers ask
1. `cv.wait(lk)` without a predicate: spurious wakeups, and a notify before the wait is lost.
2. `shutdown()` with `notify_one`: the other consumers sleep forever. Use `notify_all`.
3. Holding the queue lock while processing (ICP) blocks the producer, so sensor data gets dropped.
4. `lower_bound` result dereferenced at `end()`; `it - 1` at `begin()`.
5. Lerping quaternions without normalizing; ignoring the `q` / `-q` sign (slerp takes the long way round).
6. LRU eviction: `pop_back()` *before* erasing its key from the map, which reads a dead node.
7. `std::priority_queue` is a **max**-heap by default; a min-heap needs `std::greater<T>`.
8. Recursive flood fill on a 10⁶-cell map overflows the stack. Marking visited on *pop* duplicates queue entries.
9. `std::vector<bool>` for grids: a proxy reference, no `bool*`, and writes to neighbouring cells race.
10. Ring buffer `head - size` with `size_t` underflows. Add `N` before taking `%`.

## Robotics angle
- **IMU ring buffer**: keep the last ~1 s of samples to integrate between lidar/camera stamps.
- **Queues** decouple driver threads from estimators. Bounded + drop-oldest keeps latency bounded.
- **Timestamp lookup** for deskewing lidar sweeps and for associating camera frames with odometry (TF buffer).
- **LRU** for map tiles / submaps streamed from disk during long-range localization.
- **Running median / Hampel** filters reject range spikes; MAD gives a robust scale for Huber kernels.
- **Components** tell you if a goal is reachable before planning, and help with frontier clustering in exploration.

## Rapid-fire questions
<details><summary>Show answers</summary>

- *Why a predicate in `cv.wait`?* It re-checks the condition after spurious or stolen wakeups and handles a notify that arrived before the wait.
- *`notify_one` vs `notify_all`?* One for each item pushed; all for state changes every waiter must see (shutdown).
- *Why `list` + `unordered_map` for LRU?* Hash for O(1) find; list for O(1) reorder via `splice` with stable iterators.
- *Sliding-window median?* `multiset` plus an iterator to the median, or two heaps with lazy deletion. O(log W) per step.
- *Why avoid recursion in flood fill?* Depth can reach R·C, which overflows a ~8 MB stack.
- *Power-of-two ring capacity?* Index with `& (N-1)`; free-running counters can wrap naturally.
- *What makes SPSC lock-free safe?* Only one writer per index plus acquire/release ordering on head and tail.
- *Data race vs race condition?* Data race: unsynchronized conflicting access, which is UB. Race condition: a logic bug that depends on timing.
</details>
