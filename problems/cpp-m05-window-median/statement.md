# Sliding-Window Median Range Filter

A single-beam rangefinder on a drone produces spikes (dust, specular returns). A mean filter gets dragged around by the spikes, so you use a **median over the last `k` readings**. Implement it as a streaming filter.

```cpp
class RangeMedianFilter {
 public:
  explicit RangeMedianFilter(std::size_t k);   // k >= 1
  double push(double r);                       // add a reading, return the current median
  std::size_t size() const;                    // readings in the window: min(k, pushed so far)
};

std::vector<double> medianFilter(const std::vector<double>& ranges, std::size_t k);
```

### Semantics
- The window holds the last `min(k, count)` readings.
- Median of `n` values: the middle element when `n` is odd, and the **average of the two middle elements** when `n` is even.
- `medianFilter` returns `out[i]` = the value `push(ranges[i])` would return on a fresh filter.
- Readings repeat a lot (quantised sensor), so duplicates must be handled correctly.

### Constraints
- Up to 10⁵ readings, `k` up to a few thousand. Target **O(log k)** per push (O(k) passes the tests, but say what's wrong with it).

### Think about
- `multiset::erase(value)` vs `multiset::erase(iterator)`: what does each remove?
- Why can't a `std::priority_queue` pair (the classic streaming-median answer) handle the *sliding* window directly? What's "lazy deletion"?
- Which `multiset` iterators stay valid across inserts and erases?
- For 8-bit or mm-quantised data, what O(1) structure beats all of this?
