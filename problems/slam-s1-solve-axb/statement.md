# Solve $Ax = b$ Three Ways (+ Least Squares)

Every Gauss-Newton step in SLAM ends with a linear solve $H\,\delta = -g$. Show you know Eigen's dense solvers and when each one is appropriate.

| Function | Behaviour |
|---|---|
| `Eigen::VectorXd solveLLT(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)` | Cholesky. `A` is supposed to be SPD. If the factorization **fails** (A not positive definite), return an **empty** vector (`size() == 0`). |
| `Eigen::VectorXd solveQR(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)` | Column-pivoting Householder QR, square `A`. |
| `Eigen::VectorXd solveSVD(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)` | SVD. For rank-deficient `A` it must return the **minimum-norm** least-squares solution. |
| `Eigen::VectorXd leastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)` | `A` is m×n with m > n and full column rank: return $\arg\min_x \lVert Ax-b\rVert_2$, accurate even when $\kappa(A)\approx 10^7$. |
| `double residual(const Eigen::MatrixXd& A, const Eigen::VectorXd& x, const Eigen::VectorXd& b)` | $\lVert Ax-b\rVert_2$ |

### Constraints
- n ≤ 50, m ≤ 200. Do **not** use `A.inverse()`.

### Think about
- Cost of each method? Which would you use for the normal equations of a bundle adjustment problem?
- Why does solving $A^TA x = A^Tb$ lose accuracy? What is $\kappa(A^TA)$?
- When would you choose LDLT over LLT?
- With a near-singular A (condition number $10^{12}$), which solver gives the most sensible answer?
- Large sparse SLAM problems: `SimplicialLDLT`, CHOLMOD, fill-in, and variable ordering.
