# Robust Line Fit with Huber IRLS

A wall-following robot fits a line $y = m x + c$ to lidar returns, but about 10% of the returns hit a person walking by. Ordinary least squares gets dragged towards those outliers. Make the fit robust with a **Huber kernel** solved by **iteratively re-weighted least squares (IRLS)**.

| Function | Behaviour |
|---|---|
| `double huberWeight(double r, double k)` | IRLS weight $w(r)$ of the Huber kernel with threshold $k>0$ |
| `double madSigma(std::vector<double> r)` | Robust scale $1.4826\cdot\mathrm{median}_i\,|r_i - \mathrm{median}(r)|$; 0 for empty input |
| `Eigen::Vector2d fitLineLS(const std::vector<Eigen::Vector2d>& pts)` | Ordinary least squares $(m, c)$ |
| `Eigen::Vector2d fitLineHuber(const std::vector<Eigen::Vector2d>& pts, double k, int iters = 20, std::vector<double>* weights = nullptr)` | IRLS with Huber threshold `k`, starting from the LS fit. If `weights` is non-null, fill it with the final per-point weights (size = `pts.size()`) |

Points are $(x_i, y_i)$ and residuals are vertical: $r_i = y_i - (m x_i + c)$.

### Requirements
- Median of an even-sized set = mean of the two middle values.
- **Degenerate input** means fewer than 2 points, or a singular normal-equation system (e.g. all $x_i$ equal). Both fits then return $(0, 0)$. `weights`, if given, still gets `pts.size()` entries.
- Each IRLS iteration is O(n). Stop early once the parameters change by less than $10^{-12}$.

### Think about
- Why is the Huber weight $k/|r|$? Derive it from $\rho(r)$ via $w = \rho'(r)/r$.
- How do you choose $k$? (≈ 1.345 σ, with σ from the MAD of the LS residuals.) Why not the standard deviation?
- Huber vs Cauchy vs Tukey: which ones are convex, and which ones can reject an outlier completely?
- When would you use RANSAC instead of (or before) a robust kernel?
