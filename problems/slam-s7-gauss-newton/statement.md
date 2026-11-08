# Gauss-Newton Curve Fit

Before writing a pose-graph back-end, an interviewer often asks you to code the GN loop on a toy problem. Fit

$$y = \exp(a x^2 + b x + c)$$

to $N$ noisy samples $(x_i, y_i)$ by minimising $F(a,b,c) = \tfrac12\sum_i r_i^2$ with $r_i = y_i - \exp(a x_i^2 + b x_i + c)$. Use **Gauss-Newton with an analytic Jacobian**. You may not use numerical differentiation or an external solver.

```cpp
struct GNResult {
  Eigen::Vector3d params = Eigen::Vector3d::Zero();  // (a, b, c)
  int iterations = 0;       // number of GN steps actually applied
  bool converged = false;   // true if the step norm fell below 1e-10 before max_iters
  double final_cost = 0.0;  // 0.5 * sum r_i^2 at the returned params
};

GNResult fitExpQuadratic(const std::vector<double>& x, const std::vector<double>& y,
                         const Eigen::Vector3d& init = Eigen::Vector3d::Zero(), int max_iters = 50);
```

### Requirements
- Start from `init` (the tests use $(0,0,0)$). At each iteration form $H = J^\top J$ and $g = J^\top r$, solve $H\Delta = -g$, and apply $\Delta$.
- From $(0,0,0)$ the *full* GN step overshoots badly, because $\exp$ is very non-linear. Try it and watch $c$ run off. Add a simple **backtracking** safeguard: halve $\Delta$ until the cost does not increase (at most ~30 halvings), then apply it.
- Stop when $\|\Delta\| < 10^{-10}$ (then `converged = true`) or after `max_iters` steps.
- If `x.size() != y.size()` or there are fewer than 3 samples, return `init`, `iterations = 0`, `converged = false`, and the cost at `init`, which is 0 for empty input.
- The test data (true params $a=1, b=2, c=1$, $x\in[0,1]$, Gaussian noise on $y$) converges from zero in fewer than 20 iterations.
- O(N) work per iteration. Don't allocate an $N\times3$ Jacobian.

### Think about
- How do you detect convergence robustly (step norm, relative cost decrease, gradient norm)?
- When does Gauss-Newton diverge? What does Levenberg-Marquardt change ($\lambda$ ×10 on a rejected step, ÷10 on an accepted one)?
- Why is $J^\top J$ only an *approximation* of the Hessian, and when is that approximation bad?
- How would you get the covariance of the fitted parameters?
