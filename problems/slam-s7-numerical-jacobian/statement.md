# Numerical Jacobian Checker

Wrong Jacobians are the #1 bug in SLAM back-ends, and every team keeps a finite-difference checker around. Write one, then use it to verify the Jacobian of rotating a point under a **right perturbation** on SO(3).

| Function | Behaviour |
|---|---|
| `Eigen::MatrixXd numericalJacobian(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f, const Eigen::VectorXd& x, double eps = 1e-6)` | $m\times n$ Jacobian of $f:\mathbb{R}^n\to\mathbb{R}^m$ at $x$ using **central differences** |
| `Eigen::Matrix3d skew(const Eigen::Vector3d& v)` | $[v]_\times$, so that `skew(a) * b == a.cross(b)` |
| `Eigen::Matrix3d rotatePointJacobianAnalytic(const Eigen::Matrix3d& R, const Eigen::Vector3d& p)` | $\dfrac{\partial\, R\,\mathrm{Exp}(\delta)\,p}{\partial\delta}\Big|_{\delta=0}$ in closed form |
| `Eigen::Matrix3d rotatePointJacobianNumerical(const Eigen::Matrix3d& R, const Eigen::Vector3d& p, double eps = 1e-6)` | The same Jacobian computed with your `numericalJacobian` |

`expSO3(w)` (Rodrigues' formula) is provided in the starter.

### Requirements
- `numericalJacobian` must work for any $m, n \ge 1$. It may call `f` only $2n + 1$ times or fewer.
- The perturbation is applied **on the manifold**, $R \leftarrow R\,\mathrm{Exp}(\delta)$. Do not perturb the 9 entries of $R$.
- Analytic and numerical Jacobians should agree to about $10^{-8}$.

### Think about
- Why is the central-difference error $O(\epsilon^2)$ and forward-difference $O(\epsilon)$? Why not just use $\epsilon = 10^{-15}$?
- What is the Jacobian for a **left** perturbation $\mathrm{Exp}(\delta)R\,p$? And with respect to $p$?
- How would you unit-test every factor's Jacobian in a back-end automatically?
