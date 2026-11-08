# Question Bank — SPOILERS (interviewer only)

> Parth: don't read this file. Claude uses it to run sessions.

Format per question: **ID · Title · Difficulty (E/M/H) · Target minutes**
- **Ask:** what Claude says (statement + signature only)
- **Clarify:** good clarifying questions the candidate *should* ask (count toward Comms)
- **Hints:** escalating, give one at a time
- **Hidden tests:** edge cases Claude adds after "done"
- **Follow-ups:** for the last 10 min / deeper probe
- **Key points:** what a strong answer contains

Warmup (verbal) questions per session are at the end of each section (W-x).

---

## Session 1 — C++ & Eigen fundamentals

### 1.1 · skew-symmetric matrix and its inverse · E · 5
- **Ask:** `Eigen::Matrix3d skew(const Eigen::Vector3d& v);` and `Eigen::Vector3d vee(const Eigen::Matrix3d& S);` Show `skew(a)*b == a.cross(b)`.
- **Hints:** (1) Write out a×b component-wise. (2) Row 0 is [0, -z, y].
- **Hidden tests:** zero vector; vee(skew(v)) == v; skew(v)^T == -skew(v).
- **Follow-ups:** Where does skew show up in SLAM? (SO3 exp, Jacobians of R*p wrt rotation: -R[p]x). Why pass by const ref vs value for fixed-size Eigen (alignment rules pre-C++17)?
- **Key points:** comma initializer `S << ...`; const-ref args; fixed-size types.

### 1.2 · Solve Ax=b three ways · E · 8
- **Ask:** Given an SPD `A` (n×n) and `b`, solve with LLT, ColPivHouseholderQR, and JacobiSVD. Also solve an overdetermined least-squares system `A (m×n), m>n`. Report residuals.
- **Clarify:** is A SPD? square? conditioning?
- **Hints:** (1) `A.llt().solve(b)`. (2) Normal equations vs QR for least squares.
- **Hidden tests:** near-singular A (use condition number 1e12) — which one fails?
- **Follow-ups:** cost of each; why avoid `A.inverse()`; why normal equations square the condition number; when LDLT over LLT (semi-definite); sparse solvers for SLAM (Cholmod, `SimplicialLDLT`), fill-in, variable ordering.
- **Key points:** LLT fastest needs SPD; QR robust; SVD most robust/slowest; check `info()`.

### 1.3 · Eigen gotchas (bug hunt) · E · 8
- **Ask:** Give this code and ask what's wrong:
  ```cpp
  struct Pose { Eigen::Vector4d q; Eigen::Vector3d t; };
  std::vector<Pose> poses;                 // (a)
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(3,3);
  A = A.transpose();                       // (b)
  auto C = A * A;                          // (c) then A changes, C is used later
  Eigen::Matrix3d M; M.block<2,2>(0,0) = Eigen::Matrix2d::Identity(); // (d) rest?
  Eigen::Vector3d v = Eigen::Vector3d::Zero(); v(3) = 1;               // (e)
  ```
- **Key points:** (a) alignment of fixed-size vectorizable members in std containers pre-C++17 (`aligned_allocator`, `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`); C++17 aligned new fixes it. (b) aliasing → `transposeInPlace()`. (c) `auto` holds an expression template — lazy evaluation dangling/re-evaluated; use `.eval()` or explicit type. (d) M uninitialized elsewhere. (e) out of bounds — asserts only in debug.
- **Follow-ups:** `noalias()` — when it's safe; expression templates benefit.

### 1.4 · RAII buffer, rule of 5 · M · 12
- **Ask:** Implement `class FloatBuffer` owning a raw `float*` of size n: ctor, dtor, copy ctor/assign, move ctor/assign, `operator[]`, `size()`. No std::vector inside.
- **Hints:** (1) copy-and-swap. (2) moved-from state must be valid (nullptr, 0).
- **Hidden tests:** self-assignment; move then destroy both; copy of empty buffer.
- **Follow-ups:** `noexcept` on move and why std::vector cares (strong exception guarantee on realloc); rule of zero with `std::unique_ptr<float[]>`; when to `= delete` copy (e.g., GPU buffer, sensor handle).
- **Key points:** no double free, self-assign safe, noexcept moves, const-correct `operator[]` overloads.

### 1.5 · Smart pointer ownership in a map · M · 8
- **Ask:** Design classes `Map`, `Keyframe`, `MapPoint`. Keyframes observe map points, map points know which keyframes see them. Choose pointer types; write the members + `addObservation`.
- **Key points:** cycle → `shared_ptr` one way + `weak_ptr` back (or ids/indices into owning containers — often best). `unique_ptr` for single owner. Discuss thread safety of shared_ptr refcount vs pointee.
- **Follow-ups:** why ORB-SLAM style raw-pointer graphs are risky; why ID-based arenas are cache-friendly; `make_shared` single allocation.

### 1.6 · Templates: generic clamp/interp · E · 6
- **Ask:** Write `template<typename T> T lerp(const T& a, const T& b, double t)` that works for double and `Eigen::Vector3d`. Then a function `interpolateAt(const std::vector<std::pair<double,T>>& samples, double t)` (samples sorted by time).
- **Hints:** `std::lower_bound` with comparator on `.first`.
- **Hidden tests:** t before first / after last (clamp or error — they should ask); exactly on a sample; single sample.
- **Follow-ups:** why lerp is wrong for rotations (→ slerp); `if constexpr`; concepts.

### Warmups W-1
- Difference between `Matrix3d` and `MatrixXd`? (stack vs heap, compile-time size, unrolled ops)
- What is `const` on a member function? `mutable`?
- `std::vector::reserve` vs `resize`?
- What does `std::move` actually do? (cast to rvalue, no moving)
- Column-major vs row-major: Eigen default and why it matters for loops.

---

## Session 2 — Rotations

### 2.1 · Rotation matrix ↔ quaternion (no Eigen::Quaternion ctor) · M · 12
- **Ask:** `Eigen::Vector4d rotToQuat(const Eigen::Matrix3d& R)` returns (w,x,y,z), and `Matrix3d quatToRot(const Vector4d& q)`. Don't use Eigen's conversion.
- **Hints:** (1) trace = 4w²−1 → w. (2) If trace ≤ 0, pick largest diagonal element branch (Shepperd's method).
- **Hidden tests:** 180° rotation about x (trace=-1 → naive divide by ~0); identity; non-normalized q input; compare against Eigen.
- **Follow-ups:** double cover q ≡ −q; enforce w≥0; Hamilton vs JPL convention; memory layout of `Eigen::Quaterniond` coeffs (x,y,z,w!) vs constructor (w,x,y,z).
- **Key points:** numerically stable branch; normalization.

### 2.2 · SO(3) exp and log · M · 15
- **Ask:** `Matrix3d expSO3(const Vector3d& w)` and `Vector3d logSO3(const Matrix3d& R)`.
- **Hints:** (1) Rodrigues: R = I + sinθ/θ K + (1−cosθ)/θ² K², K = skew(w). (2) log: θ = acos((tr−1)/2), w = θ/(2 sinθ) vee(R−Rᵀ).
- **Hidden tests:** w = 0 and |w| = 1e-9 (Taylor expansion needed); θ ≈ π (sinθ ≈ 0 in log — use (R+I)/2 = aaᵀ); round trip for random w with |w|<π; clamp acos argument to [-1,1].
- **Follow-ups:** Why optimize on tangent space (minimal params, no constraint)? Right vs left perturbation; right Jacobian of SO3; BCH.
- **Key points:** small-angle handling, π handling, clamping.

### 2.3 · Euler angles pitfalls · E · 8
- **Ask:** Convert ZYX (yaw-pitch-roll) to R and back. Then: what happens at pitch = ±90°?
- **Key points:** `R = Rz(yaw)*Ry(pitch)*Rx(roll)`; extraction via atan2; gimbal lock → yaw and roll coupled; never interpolate/average Euler angles. Note `R.eulerAngles(2,1,0)` returns ranges that can surprise ([0,π] for first).
- **Hidden tests:** pitch = 89.9999°, wrap-around yaw at ±π.

### 2.4 · Slerp · M · 10
- **Ask:** `Quaterniond slerp(Quaterniond a, Quaterniond b, double t)` without Eigen's slerp.
- **Hints:** (1) dot<0 → negate b (shortest path). (2) dot≈1 → fall back to nlerp.
- **Hidden tests:** a == b; a == −b; t=0/1; 179° apart; compare to `a.slerp(t,b)`.
- **Follow-ups:** equivalent formulation a·exp(t·log(a⁻¹b)); interpolating poses between lidar scans for motion-compensation (deskewing).

### 2.5 · Re-orthonormalize drifting rotation · E · 6
- **Ask:** After many integrations R isn't orthonormal anymore. Write `Matrix3d nearestRotation(const Matrix3d& M)`.
- **Key points:** SVD: R = U Vᵀ, fix det = −1 by flipping last column of U; alternatives: Gram-Schmidt (biased to first axis), quaternion renormalize.
- **Hidden tests:** M with det<0; M = 2·R.

### 2.6 · Rotation averaging · H · 15
- **Ask:** Given N noisy quaternions of the same orientation, return the mean.
- **Key points:** naive component average fails due to sign; align signs to first then normalize (ok for small spread); proper: eigenvector of max eigenvalue of Σ qqᵀ (Markley); or iterative Karcher mean via log/exp.
- **Follow-ups:** weighted; outliers → robust (median/Huber in tangent space).

### Warmups W-2
- Why are quaternions preferred over Euler for state? Over rotation matrices?
- How many DOF does a rotation have; how many params does R have; what are the constraints?
- What is the double cover?
- Active vs passive rotation; `R_wb` naming convention — what does `p_w = R_wb * p_b` mean?
- What's det(R) for a reflection?

---

## Session 3 — Rigid transforms (SE3)

### 3.1 · SE3 struct · E · 10
- **Ask:** Implement `struct SE3 { Matrix3d R; Vector3d t; }` with `operator*` (compose), `inverse()`, `operator*(Vector3d)`, `Matrix4d matrix()`.
- **Hidden tests:** T*T.inverse() == I; associativity; inverse without 4x4 inversion (Rᵀ, −Rᵀt).
- **Follow-ups:** `Eigen::Isometry3d` vs `Affine3d` (inverse cost); storage as quaternion+t.

### 3.2 · Batch transform point cloud · E · 8
- **Ask:** Transform `Matrix3Xd P` by SE3 efficiently; also do it for `std::vector<Vector3d>`. Measure time for 1M points (`./run.sh -O`).
- **Key points:** `(R*P).colwise() + t`; avoid per-point allocations; `reserve`; Map over raw float buffer (`Eigen::Map<Matrix3Xf>`) for zero-copy sensor data.
- **Follow-ups:** float vs double; SIMD; row-major Nx3 layout from sensors; `Eigen::Map` with stride (x,y,z,intensity).

### 3.3 · Frame chain / relative pose · M · 10
- **Ask:** Given `T_world_robot`, `T_robot_lidar`, `T_world_camera`, express a lidar point in the camera frame. Then: given two robot poses T_w_a and T_w_b, compute relative motion T_a_b, and the odometry noise-free prediction of b given a and T_a_b.
- **Key points:** subscript cancellation T_c_l = T_w_c⁻¹ T_w_r T_r_l; precise naming.
- **Hidden tests:** catches swapped order.

### 3.4 · 2D pose (SE2) composition & odometry integration · E · 10
- **Ask:** Integrate wheel odometry (v, ω, dt) samples into a 2D pose (x, y, θ). Handle angle wrapping.
- **Key points:** exact arc integration vs Euler; ω≈0 branch; `atan2(sin, cos)` normalization.
- **Follow-ups:** covariance propagation (Jacobians F, G) → leads to EKF session.

### 3.5 · Plane / line transformation · M · 8
- **Ask:** A plane (n, d) with n·x + d = 0 is in frame A. Transform it to frame B given T_B_A.
- **Key points:** n' = R n, d' = d − n'·t. Lines via Plücker (bonus).

### Warmups W-3
- How many params in SE3, DOF?
- What does `T_a_b` mean in your convention?
- Why can't you just average two 4x4 matrices?
- Homogeneous coordinates: what does w=0 represent?

---

## Session 4 — Point cloud basics

### 4.1 · Voxel grid downsample · M · 15
- **Ask:** `std::vector<Vector3d> voxelDownsample(const std::vector<Vector3d>& pts, double leaf)` returning centroid per occupied voxel.
- **Clarify:** centroid vs first point vs closest-to-center; negative coordinates; memory budget.
- **Hints:** (1) voxel index = floor(p/leaf) (not cast — negatives!). (2) hash a 3-int key into `unordered_map`.
- **Hidden tests:** negative coords (−0.01 and 0.01 must be different voxels); empty cloud; all points in one voxel; leaf ≤ 0.
- **Follow-ups:** hash function quality (e.g. XOR of primes 73856093, 19349663, 83492791); sort-based alternative (compute key, sort, run-length reduce) — deterministic and cache friendly; O(N) vs O(N log N); keep intensity/attributes; streaming / incremental voxel map (ikd-tree, iVox).
- **Key points:** `std::floor`, custom hasher struct, accumulate sum+count, reserve.

### 4.2 · Brute-force kNN · E · 8
- **Ask:** `std::vector<int> knn(const Cloud& pts, const Vector3d& q, int k)` returning indices sorted by distance.
- **Key points:** squared distance; `std::nth_element`/`partial_sort` or max-heap of size k (O(N log k)); k > N.
- **Follow-ups:** complexity; when brute force beats a tree (small N, GPU).

### 4.3 · Centroid & covariance; normal estimation · M · 15
- **Ask:** For each point, estimate a surface normal from its k nearest neighbors (use your brute-force kNN). Orient normals toward a sensor origin.
- **Hints:** (1) covariance of neighbors. (2) `SelfAdjointEigenSolver`, smallest eigenvalue's eigenvector.
- **Hidden tests:** points on plane z=0 → normals ±z; collinear neighbors (degenerate) — detect via eigenvalue ratios; fewer than 3 neighbors.
- **Follow-ups:** curvature = λ0/(λ0+λ1+λ2); numerically: two-pass covariance vs one-pass (catastrophic cancellation far from origin) — subtract centroid; planarity / linearity features (LOAM edge vs plane).
- **Key points:** SelfAdjointEigenSolver sorts eigenvalues ascending; flip sign if n·(origin−p) < 0.

### 4.4 · Statistical outlier removal · M · 12
- **Ask:** Remove points whose mean distance to k neighbors > mean + α·std over the cloud.
- **Hidden tests:** cloud with a few far isolated points; all identical points (std = 0).
- **Follow-ups:** radius outlier removal; complexity with KD-tree; ROR vs SOR for rain/dust in lidar.

### 4.5 · Range image projection · M · 12
- **Ask:** Given a spinning lidar with 32 rings (vertical FOV −30°..+10°) and 1800 horizontal bins, project points into a 32×1800 range image (keep nearest per pixel).
- **Key points:** azimuth atan2(y,x), elevation atan2(z, sqrt(x²+y²)); bounds checks; range images enable fast neighbor search, ground segmentation, ring-based feature extraction.

### 4.6 · Ground removal (simple) · E · 8
- **Ask:** Remove ground points: points with z below a threshold relative to the sensor height, then discuss why that fails on slopes.
- **Follow-ups:** RANSAC plane (→ 5.2), grid-based min-z, Patchwork-style concentric zones.

### Warmups W-4
- Why voxelize before ICP?
- What's an organized vs unorganized point cloud?
- Why use squared distances?
- What's the memory of 1M points as `vector<Vector3d>` vs `vector<Vector3f>`? (24MB vs 12MB; Vector4f padding for SSE)

---

## Session 5 — Spatial search & robust fitting

### 5.1 · KD-tree · H · 25
- **Ask:** Build a 3D KD-tree over `std::vector<Vector3d>`: `build`, `nearest(q) -> index`, `radiusSearch(q, r) -> vector<int>`.
- **Clarify:** static or dynamic? duplicates? return distances?
- **Hints:** (1) split on axis = depth%3 or max-spread axis, median via `nth_element` on index array. (2) search near child first, visit far child only if (q[axis]-split)² < best.
- **Hidden tests:** compare vs brute force on 10k random queries; duplicate points; single point; query far outside bounds; r = 0.
- **Follow-ups:** O(N log N) build, avg O(log N) query, worst-case; leaf buckets (10–20 points) for cache; nodes in a flat vector vs pointers; approximate NN (nanoflann, FLANN); incremental insert (ikd-tree rebalancing); why KD-trees degrade in high dims.
- **Key points:** recursion with index ranges, pruning test correct, no copying points.

### 5.2 · RANSAC plane fitting · M · 15
- **Ask:** `bool ransacPlane(const Cloud& pts, double thresh, int iters, Vector4d& plane, std::vector<int>& inliers)`.
- **Hints:** (1) 3 random distinct points → n = (p2−p1)×(p3−p1). (2) skip degenerate (|n|≈0).
- **Hidden tests:** 70% plane + 30% noise; all collinear points; fewer than 3 points; repeatable with seeded rng.
- **Follow-ups:** iterations N = log(1−p)/log(1−wˢ); refit with least squares (PCA) on inliers; adaptive termination; PROSAC/MSAC/LO-RANSAC; multiple planes (remove inliers & repeat).

### 5.3 · Least-squares plane / line fit (SVD) · E · 8
- **Ask:** Fit a plane to points minimizing orthogonal distance. Then a 3D line.
- **Key points:** centroid + smallest-eigvec (plane) / largest-eigvec (line); why not z = ax+by+c (vertical planes, not orthogonal error).

### 5.4 · Euclidean clustering with union-find · M · 15
- **Ask:** Cluster points so that any two within distance r are in the same cluster. Return cluster ids. Implement union-find yourself.
- **Key points:** path compression + union by rank; neighbors via voxel hash with leaf = r checking 27 cells (or KD-tree radius search); BFS alternative.
- **Hidden tests:** chain of points spaced 0.9r (one cluster); min/max cluster size filter.

### 5.5 · Circle / sphere fit · M · 12
- **Ask:** Fit a sphere center & radius to points (algebraic least squares). Then RANSAC wrapper.
- **Key points:** linearize |p|² = 2c·p + (r² − |c|²) → linear in (c, k).

### Warmups W-5
- KD-tree vs octree vs voxel hash — when each?
- Why RANSAC rather than least squares with outliers?
- What's the probability formula for RANSAC iterations?

---

## Session 6 — Registration

### 6.1 · Kabsch / Umeyama · M · 15
- **Ask:** Given corresponding point sets P, Q (3×N), find R, t minimizing Σ|R pᵢ + t − qᵢ|².
- **Hints:** (1) center both. (2) H = Σ p̃ q̃ᵀ, SVD H = U S Vᵀ, R = V Uᵀ. (3) reflection.
- **Hidden tests:** noise-free random pose recover to 1e-9; reflection case (det = −1) handled via diag(1,1,det(VUᵀ)); N = 3 points; collinear points (degenerate — rank deficient); with scale (Umeyama).
- **Follow-ups:** weighted version; compare with `Eigen::umeyama`; why closed form exists (orthogonal Procrustes).

### 6.2 · Point-to-point ICP · H · 20
- **Ask:** `SE3 icp(const Cloud& src, const Cloud& tgt, SE3 init, int max_iters)` using your KD-tree (or brute force if short on time) and Kabsch.
- **Clarify:** max correspondence distance? convergence criterion?
- **Hidden tests:** small random pose (10°, 0.2m) converges; 90° fails (discuss local minimum); partial overlap with distance gating; src == tgt.
- **Follow-ups:** convergence criteria (Δpose, Δerror); outlier rejection (distance threshold, trimmed ICP, robust kernels); coarse-to-fine voxel pyramid; degeneracy in corridors (eigenvalues of Hessian); real-time budget at 10 Hz.

### 6.3 · Point-to-plane ICP (one GN step) · H · 20
- **Ask:** Given correspondences (pᵢ, qᵢ, nᵢ), write one Gauss-Newton step: minimize Σ (nᵢ·(R pᵢ + t − qᵢ))² with small-angle linearization R ≈ I + [ω]×.
- **Key points:** residual rᵢ = nᵢ·(pᵢ − qᵢ) (at current est), Jacobian Jᵢ = [ (pᵢ × nᵢ)ᵀ, nᵢᵀ ] (1×6); H = Σ JᵀJ, b = Σ Jᵀr, δ = −H⁻¹b; update with exp(ω).
- **Follow-ups:** why converges faster than p2p; GICP (plane-to-plane); degeneracy detection via eigenvalues of H (LOAM-style "solution remapping").

### 6.4 · Scan-to-map odometry design (discussion + skeleton) · H · 15
- **Ask:** Sketch the classes for a lidar odometry pipeline: deskew → downsample → register to local map → update map. Write the header/interfaces only.
- **Key points:** sliding local map (voxel hash), keyframe selection by distance/angle, constant-velocity prior as init, threading (producer/consumer of scans). Compare to KISS-ICP / FAST-LIO.

### Warmups W-6
- Why does ICP need a good initial guess?
- Point-to-point vs point-to-plane vs NDT/GICP?
- How do you detect a degenerate registration?

---

## Session 7 — Nonlinear least squares

### 7.1 · Gauss-Newton curve fit · M · 15
- **Ask:** Fit y = exp(a x² + b x + c) to noisy data with Gauss-Newton (analytic Jacobian). Init (0,0,0).
- **Hidden tests:** recover true params within tol; converge in < 20 iters; noise σ=1.
- **Follow-ups:** convergence check; when GN diverges; Levenberg-Marquardt damping update rule (λ ×10 / ÷10 on fail/success, accept/reject step).

### 7.2 · Numerical Jacobian checker · E · 8
- **Ask:** Write a generic `numericalJacobian(f, x, eps)` using central differences for `f: VectorXd -> VectorXd`. Use it to verify the analytic Jacobian of `f(R, p) = R p` wrt a right perturbation on R.
- **Key points:** central differences error O(eps²); eps ~ 1e-6 for doubles; for manifolds perturb via exp, not raw entries. Answer: ∂(R exp(δ) p)/∂δ = −R [p]×.

### 7.3 · Robust kernel (Huber / Cauchy) via IRLS · M · 12
- **Ask:** Modify 7.1 or line fit to use Huber weights with 10% outliers.
- **Key points:** weights w(r) = 1 if |r|≤k else k/|r|; weighted normal equations JᵀWJ; scale estimation (MAD).

### 7.4 · 2D pose graph optimization · H · 25
- **Ask:** Poses xᵢ = (x,y,θ). Edges (i,j, z_ij measured relative pose, information Ω). Implement GN: residual e = log( z⁻¹ · (xᵢ⁻¹ xⱼ) ) (use simple SE2 residual), assemble sparse-ish H (dense ok for ≤50 poses), fix x₀ (gauge), iterate. Test on a square loop with drifting odometry + 1 loop closure.
- **Hints:** (1) residual in SE2: [Rᵢᵀ(tⱼ−tᵢ) − t_z rotated..., wrap(θⱼ−θᵢ−θ_z)]. (2) Jacobians Aᵢⱼ, Bᵢⱼ (Grisetti tutorial). (3) Gauge: add large prior on x₀ or remove its block.
- **Follow-ups:** sparsity structure; Schur complement (for BA); why Cholesky + reordering (COLAMD/AMD); iSAM2 incremental; switchable constraints for false loop closures.

### 7.5 · Marginalization / Schur complement · H · 10 (whiteboard)
- **Ask:** Given H = [[A, B],[Bᵀ, C]] and b, eliminate the second block. Write it with Eigen.
- **Key points:** H' = A − B C⁻¹ Bᵀ, b' = b_a − B C⁻¹ b_c; fill-in; sliding-window VIO; FEJ.

### Warmups W-7
- GN vs LM vs gradient descent?
- What does the information matrix represent?
- Why does BA have a block-sparse arrow structure?
- What is a gauge freedom?

---

## Session 8 — Estimation & mapping

### 8.1 · EKF for a 2D robot with known landmarks · H · 25
- **Ask:** State (x,y,θ). Predict with (v, ω, dt) unicycle; update with range-bearing measurement to a known landmark. Implement `predict` and `update` with Jacobians and covariances.
- **Hidden tests:** bearing wrap-around (innovation near ±π must be wrapped!); covariance stays symmetric PD (use Joseph form or symmetrize); zero ω.
- **Follow-ups:** Mahalanobis gating for data association; EKF-SLAM state augmentation and O(n²) cost; inconsistency / linearization errors; UKF / error-state EKF for IMU (why error-state).

### 8.2 · Pinhole projection with distortion · E · 10
- **Ask:** Project 3D points in camera frame to pixels with K and radial-tangential (k1,k2,p1,p2). Handle points behind camera.
- **Follow-ups:** undistortion (iterative inverse); Jacobian of projection wrt point (2×3) — used in BA.

### 8.3 · Triangulation · M · 15
- **Ask:** Given two camera poses and normalized image observations of the same point, triangulate (DLT with SVD). Also: midpoint method.
- **Hidden tests:** noise-free exact; small baseline (ill-conditioned) — discuss parallax angle check; point behind cameras (cheirality).

### 8.4 · Occupancy grid with Bresenham · M · 20
- **Ask:** 2D grid (resolution, origin). For each lidar beam from robot pose, mark cells along the ray free and endpoint occupied using log-odds. Implement Bresenham yourself.
- **Hidden tests:** steep / negative slopes; beam ending outside the map; max-range readings (no hit — free only); clamping log-odds.
- **Follow-ups:** log-odds rationale (additive, avoid saturation); 3D (OctoMap); dynamic objects; memory for large maps (tiled/hashed grids).

### 8.5 · A* on occupancy grid · M · 15
- **Ask:** Shortest path on an 8-connected grid from start to goal avoiding occupied cells.
- **Key points:** priority_queue with (f, idx), closed set, admissible octile heuristic, path reconstruction, no corner cutting.

### 8.6 · IMU preintegration (discussion + simple integration) · H · 15
- **Ask:** Integrate gyro + accel samples (with gravity, biases given) to propagate R, v, p. Then explain why preintegration exists.
- **Key points:** R ← R exp(ω dt); v ← v + (R a + g) dt; p ← p + v dt + ½(Ra+g)dt²; bias re-linearization via Jacobians; on-manifold preintegration (Forster).

### Warmups W-8
- KF vs EKF vs particle filter — when each?
- Why log-odds?
- What's the difference between filtering and smoothing?
- Explain loop closure end-to-end (place recognition → geometric verification → pose graph).

---

## C++ / DS&A (sprinkle 1 per session)

### C.1 · Ring buffer for IMU · E · 10
- **Ask:** Fixed-capacity `template<typename T, size_t N> class RingBuffer` with `push` (overwrite oldest), `size`, `operator[](i)` (0=oldest), `back()`.
- **Hidden tests:** wrap-around many times; N=1; access on empty.
- **Follow-ups:** lock-free SPSC version (atomics with acquire/release), power-of-two mask.

### C.2 · Thread-safe queue · M · 12
- **Ask:** `ThreadSafeQueue<T>` with `push`, blocking `pop`, `tryPop`, and `shutdown()` so consumers exit. Write a producer (lidar thread) / consumer (odometry thread) demo.
- **Key points:** mutex + condition_variable, predicate wait (spurious wakeups), notify outside lock optional, bounded version drops oldest (real-time).
- **Follow-ups:** deadlocks; priority inversion; why not hold a lock while doing ICP.

### C.3 · Nearest timestamp lookup · E · 8
- **Ask:** Given a sorted vector of (timestamp, pose) at 100 Hz, find the pose for a lidar timestamp: nearest, or interpolated (lerp t, slerp R).
- **Key points:** lower_bound, boundary handling, tolerance (reject if gap > threshold).

### C.4 · LRU cache for map tiles · M · 15
- **Ask:** `LRUCache<Key, Value>` with get/put, O(1).
- **Key points:** list + unordered_map<Key, list::iterator>, splice.

### C.5 · Top-k / streaming median · M · 10
- **Ask:** Running median of range readings (two heaps).

### C.6 · Flood fill / connected components on grid · E · 10
- **Ask:** Count connected free-space regions in a 2D occupancy grid (BFS, 4-connectivity).

### C.7 · Object lifetime quiz · E · 5 (verbal)
- Dangling reference from returning `const&` to local; `string_view` to temporary; iterator invalidation on `vector::push_back`; virtual destructor missing; `std::vector<bool>`.

---

## Mock interview recipes (60 min, Session 9+)
Pick 1 from each column; randomize:
| Warmup (5') | Easy/Med (15–20') | Hard (25') | Follow-up (10') |
|---|---|---|---|
| any W-x | 1.1, 2.5, 3.3, 4.1, 4.3, 5.2, 6.1, 8.2, 8.4, C.1, C.2 | 2.2, 5.1, 6.2, 6.3, 7.1, 7.4, 8.1 | system design: 6.4, loop closure, multi-sensor timing |

Likely Sunday-Robotics-flavoured combos (home robot: indoor lidar/RGB-D, manipulation):
- A: 4.1 voxel grid → 6.1 Kabsch → follow-up: ICP design
- B: 2.2 SO3 exp/log → 4.3 normals → follow-up: point-to-plane
- C: 5.2 RANSAC plane (tables/floors!) → 5.4 clustering (objects on table) → follow-up: grasp frame from cluster PCA
- D: 3.3 frame chains (camera on arm, hand-eye) → 8.3 triangulation → 8.2 projection
- E: C.2 thread-safe queue → 8.4 occupancy grid
