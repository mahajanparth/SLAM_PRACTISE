# Matrix Class: Rule of Five

The interviewer says: "No Eigen. Write a small dense matrix class for 6×6 covariance math, managing the memory
yourself." The point is to show the **Rule of Five**, correct move semantics, and operator overloading.

Storage is a raw owning `double* data_` (row-major) plus `rows_`, `cols_`. The constructors that allocate, the
destructor, `operator()`, and the accessors are **given**. Implement:

| Member | Behaviour |
|---|---|
| `Matrix(const Matrix& o)` | Deep copy |
| `Matrix& operator=(const Matrix& o)` | Deep copy; self-assignment safe; strong exception guarantee is a bonus |
| `Matrix(Matrix&& o) noexcept` | Steal the buffer: **no allocation**. `o` is left as a valid empty 0×0 matrix (`data() == nullptr`) |
| `Matrix& operator=(Matrix&& o) noexcept` | Same; releases the old buffer; self-move safe |
| `Matrix& operator+=(const Matrix& o)` | Element-wise; throws `std::invalid_argument` on a dimension mismatch |
| `Matrix operator+(Matrix a, const Matrix& b)` | Free function |
| `Matrix operator*(const Matrix& a, const Matrix& b)` | Matrix product; throws `std::invalid_argument` if `a.cols() != b.rows()` |
| `Matrix transposed() const` | |
| `bool operator==(const Matrix& o) const` | Same dims and exactly equal elements (two 0×0 matrices are equal) |
| `static Matrix identity(std::size_t n)` | |

The tests count how many times your code allocates via `Matrix::allocations()` (the given `allocate` helper
increments it). Moves must not allocate.

### Think about
- Why must the move constructor be `noexcept`? What does `std::vector<Matrix>` do on reallocation otherwise?
- Copy-and-swap vs. a hand-written copy assignment: exception safety and cost.
- Why take the left operand of `operator+` **by value**?
- How would `std::vector<double>` as the storage change all of this (Rule of Zero)?
- Expression templates (Eigen): what problem do they solve for `A + B + C`?
