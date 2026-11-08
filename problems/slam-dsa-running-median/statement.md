# Running Median of Range Readings

A single-beam rangefinder publishes noisy readings with occasional spikes (multipath, dust). A running
**median** is robust to those spikes where a mean is not. Implement it for an unbounded stream.

```cpp
class RunningMedian {
 public:
  void add(double x);           // O(log n)
  double median() const;        // O(1); NaN if no readings yet
  std::size_t size() const;
};
```

- Even count: the median is the **mean of the two middle values**.
- Readings may be negative or repeated. Inputs are finite (no NaN).

### Think about
- Why not sort on every query, or `std::nth_element`? What are their costs per reading?
- A **sliding window** median over the last `W` readings: what breaks with heaps? (lazy deletion, or a
  `std::multiset` with an iterator to the median)
- Top-k largest of a stream: which heap, and what size?
- A Hampel filter (median + MAD) for spike rejection.
