# Marginalization via the Schur Complement

A sliding-window VIO keeps only the last few keyframes. When the oldest one leaves the window it is **marginalised**: its information is folded into a prior on the remaining states. Bundle adjustment uses the same algebra to eliminate thousands of landmarks before solving for the cameras.

Given a symmetric positive-definite system $H\Delta = b$ partitioned as

$$\begin{bmatrix}A & B\\ B^\top & C\end{bmatrix}\begin{bmatrix}\Delta_a\\ \Delta_c\end{bmatrix} = \begin{bmatrix}b_a\\ b_c\end{bmatrix},\qquad A \in \mathbb{R}^{n_a\times n_a},\; C\in\mathbb{R}^{n_c\times n_c},$$

keep the **first** `na` variables and eliminate the rest.

```cpp
struct Reduced { Eigen::MatrixXd H; Eigen::VectorXd b; };

Reduced marginalize(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, int na);
Eigen::VectorXd solveViaSchur(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, int na);
```

| Function | Behaviour |
|---|---|
| `marginalize` | The reduced $n_a\times n_a$ system $(H', b')$ on $\Delta_a$ only |
| `solveViaSchur` | Full solution $\Delta$ (size $n$): solve the reduced system for $\Delta_a$, then back-substitute for $\Delta_c$ |

### Requirements
- $0 \le n_a \le n$. If $n_a = n$, return $(H, b)$ unchanged. If $n_a = 0$, return a $0\times0$ matrix and an empty vector.
- Don't call `.inverse()` on $C$. Factor it once and use `solve`.
- `solveViaSchur` must agree with `H.ldlt().solve(b)`.

### Think about
- In BA, $C$ (landmarks) is block-diagonal with 3×3 blocks. What does inverting it cost, and what is the size of the reduced camera system?
- Why is $H'$ generally **dense** even when $H$ is sparse (fill-in)? Why does that hurt a sliding window over time?
- Show that $H'^{-1}$ equals the top-left block of $H^{-1}$. What does that mean probabilistically?
- What is First-Estimate Jacobians (FEJ), and why does marginalisation need it?
