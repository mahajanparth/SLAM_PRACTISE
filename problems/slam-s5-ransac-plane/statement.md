# RANSAC Plane Fitting

Ground segmentation, wall extraction and table-top detection all start with "find the dominant plane despite clutter". Least squares breaks down with even one gross outlier, so use **RANSAC**.

```cpp
using Cloud = std::vector<Eigen::Vector3d>;

bool ransacPlane(const Cloud& pts, double thresh, int iters,
                 Eigen::Vector4d& plane, std::vector<int>& inliers, unsigned seed = 42);
```

- Plane is $(n_x, n_y, n_z, d)$ with $\lVert n\rVert = 1$, meaning the points satisfy $n\cdot p + d = 0$. Either sign of $n$ is accepted.
- `inliers`: **ascending** indices of all points with $|n\cdot p_i + d| \le$ `thresh`, measured against the plane you return.
- Return `true` on success. Return `false` (and leave the outputs empty or unchanged) if there are fewer than 3 points or no non-degenerate (non-collinear) sample is found within `iters` iterations.
- Use your own `std::mt19937` seeded with `seed`, so the same input and seed give the same result.

### Constraints
- Up to ~10⁴ points, `iters` up to a few hundred. Cost is $O(\text{iters}\cdot N)$.
- Samples may contain duplicate or collinear points. Skip them.

### Think about
- How many iterations do you need for inlier ratio $w$, sample size $s=3$, success probability $p$? How do you terminate adaptively?
- Why refit with least squares on the inliers, and what does LO-RANSAC add?
- MSAC/PROSAC: how do they change the scoring or the sampling?
- How do you extract several planes (walls + floor)?
