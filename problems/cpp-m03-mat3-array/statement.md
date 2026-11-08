# Fixed-size 3×3 Math with `std::array`

On a microcontroller-class IMU board you can't pull in Eigen, but you still need rotations. Write a tiny 3×3 toolkit on top of `std::array`, which is a fixed-size value type with no heap use.

```cpp
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<Vec3, 3>;   // row-major: M[row][col]
```

| Function | Behaviour |
|---|---|
| `Mat3 identity()` | 3×3 identity |
| `Mat3 transpose(const Mat3& M)` | $M^\top$ |
| `Mat3 matmul(const Mat3& A, const Mat3& B)` | $AB$. Must be correct when `A` and `B` are the same object |
| `Vec3 mulVec(const Mat3& M, const Vec3& v)` | $Mv$ (not called `apply`: ADL on `std::array` arguments would find `std::apply`) |
| `double det(const Mat3& M)` | determinant |
| `std::pair<Vec3, double> normalize(const Vec3& v)` | `{v/‖v‖, ‖v‖}`. For a zero vector, return `{{0,0,0}, 0}` |
| `std::tuple<double, std::size_t, std::size_t> maxAbsEntry(const Mat3& M)` | `{value, row, col}` of the entry with the largest absolute value (its signed value). Ties go to the first in row-major order |

### Constraints
- No heap allocation, no Eigen. Everything is O(1).

### Think about
- `sizeof(Mat3)`? Is a `Mat3` returned by value expensive? Where does it live?
- `std::array` vs a C array `double[3][3]`: what do you gain (copy, `==`, `.size()`, no decay to pointer)?
- Can you use `auto [u, n] = normalize(v);` to *modify* the pair's members? What does `auto&` change?
- When would you prefer a named struct over a `std::tuple` return?
