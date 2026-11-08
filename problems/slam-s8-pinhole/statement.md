# Pinhole Projection with Distortion

You're colorizing a lidar cloud from a calibrated camera. The points are already in the **camera frame** (z forward). Implement projection with the radial-tangential (plumb-bob, OpenCV `k1,k2,p1,p2`) model.

```cpp
struct Intrinsics { double fx, fy, cx, cy, k1, k2, p1, p2; };

// Returns false (uv untouched) if the point is not in front of the camera: Z <= min_depth.
bool projectPoint(const Intrinsics& K, const Eigen::Vector3d& Pc, Eigen::Vector2d& uv,
                  double min_depth = 1e-6);

// Projects every point; returns the indices (ascending) of points that are in front of the
// camera AND land inside the image: 0 <= u < width, 0 <= v < height. pixels[k] belongs to idx[k].
std::vector<int> projectToImage(const Intrinsics& K, const std::vector<Eigen::Vector3d>& cloud,
                                int width, int height, std::vector<Eigen::Vector2d>& pixels);
```

### Given model
With $x = X/Z$, $y = Y/Z$, $r^2 = x^2 + y^2$:
$$x_d = x(1 + k_1 r^2 + k_2 r^4) + 2p_1 x y + p_2(r^2 + 2x^2)$$
$$y_d = y(1 + k_1 r^2 + k_2 r^4) + p_1(r^2 + 2y^2) + 2p_2 x y$$
$$u = f_x x_d + c_x,\qquad v = f_y y_d + c_y$$

### Constraints
- `projectToImage` is $O(n)$ and clears or overwrites `pixels`.

### Think about
- How do you **undistort** a pixel? There's no closed form, so you need an iterative fixed point or Newton.
- Derive the $2\times3$ Jacobian $\partial(u,v)/\partial P_c$ (no distortion). Where is it used?
- Why does strong barrel distortion make the polynomial non-monotonic far outside the image, and why does that matter for wide-FOV lidar points?
