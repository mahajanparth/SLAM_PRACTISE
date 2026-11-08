# Spinning-Lidar Range Image Projection

A 32-beam spinning lidar (vertical FOV −30°…+10°, 1800 azimuth bins) produces an unorganized cloud. Project it into a **range image** so that neighbour lookups become O(1) pixel accesses (used for ground segmentation, LeGO-LOAM-style feature extraction, and fast clustering).

```cpp
struct RangeImage {
  int rows = 0, cols = 0;
  std::vector<double> range;   // rows*cols, row-major; -1 for empty pixels
  std::vector<int> index;      // rows*cols; index of the point stored in the pixel, -1 if empty
  double at(int r, int c) const { return range[r * cols + c]; }
};

RangeImage projectToRangeImage(const std::vector<Eigen::Vector3d>& pts, int rows = 32, int cols = 1800,
                               double fov_down_deg = -30.0, double fov_up_deg = 10.0);
```

### Pixel mapping (exact, so results are testable)
- $r = \lVert p\rVert$. Points with $r < 10^{-6}$ are discarded.
- Azimuth $a = \operatorname{atan2}(y, x) \in [-\pi, \pi]$, **col** $= \lfloor (a + \pi) / (2\pi) \cdot \text{cols} \rfloor$, clamped to `cols - 1`.
- Elevation $e = \operatorname{atan2}(z, \sqrt{x^2+y^2})$. Points with $e < \text{fov\_down}$ or $e > \text{fov\_up}$ are **discarded**. **row** $= \lfloor (\text{fov\_up} - e) / (\text{fov\_up} - \text{fov\_down}) \cdot \text{rows} \rfloor$ (row 0 is the top beam), clamped to `rows - 1`.
- If several points fall into one pixel, keep the **nearest** (smallest range). On equal ranges keep the earlier point.
- The returned image always has `rows*cols` entries, even when the input is empty.

### Think about
- Why is "keep nearest" the right choice (occlusion), and when would you keep the last return instead?
- Real sensors give a ring index per point. Why prefer it over computing elevation?
- How would you find the 8 neighbours of a pixel, including wrap-around at the azimuth seam?
- Ground segmentation on the range image: compare the angle between vertically adjacent pixels.
