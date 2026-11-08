// The candidate's code is injected before this file (g++ -include).
#include <type_traits>
#include <vector>
#include "common/check.h"

static bool dims(const Matrix& m, std::size_t r, std::size_t c) { return m.rows() == r && m.cols() == c; }

int main() {
  Matrix a{{1, 2}, {3, 4}};
  Matrix b = a;  // copy
  CHECK(dims(b, 2, 2));
  if (dims(b, 2, 2)) {
    b(0, 0) = 100;
    CHECK_NEAR(a(0, 0), 1.0, 0);  // deep copy: original untouched
    CHECK(b.data() != a.data());
  }

  const double* buf = a.data();
  std::size_t before = Matrix::allocations();
  Matrix m(std::move(a));
  CHECK(m.data() == buf);                       // buffer stolen
  CHECK(a.rows() == 0 && a.data() == nullptr);  // moved-from is empty
  CHECK_EQ(Matrix::allocations(), before);      // move doesn't allocate

  Matrix s = m + Matrix{{1, 1}, {1, 1}};
  CHECK(s == (Matrix{{2, 3}, {4, 5}}));
  Matrix p = Matrix{{1, 2, 3}} * Matrix{{1}, {2}, {3}};
  CHECK(dims(p, 1, 1) && p(0, 0) == 14.0);
  CHECK(Matrix::identity(3) == (Matrix{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}));
  CHECK((Matrix{{1, 2, 3}}).transposed() == (Matrix{{1}, {2}, {3}}));

#ifdef HIDDEN
  CHECK(std::is_nothrow_move_constructible_v<Matrix>);
  CHECK(std::is_nothrow_move_assignable_v<Matrix>);
  // copy assignment: replaces, deep, self-assignment safe
  Matrix c{{9}};
  c = m;
  CHECK(c == m && c.data() != m.data());
  Matrix& alias = c;
  c = alias;
  CHECK(c == m);
  // move assignment frees old buffer (ASan leak check) and leaves source empty; self-move safe
  Matrix d(3, 3, 7.0);
  before = Matrix::allocations();
  d = std::move(c);
  CHECK(d == m && c.rows() == 0 && c.cols() == 0 && c.data() == nullptr);
  CHECK_EQ(Matrix::allocations(), before);
  Matrix& dself = d;
  d = std::move(dself);
  CHECK(d == m);
  // moved-from object is reusable
  c = Matrix(2, 2, 1.0);
  CHECK(c == (Matrix{{1, 1}, {1, 1}}));
  // empty matrices
  Matrix e1, e2;
  CHECK(e1 == e2);
  Matrix e3 = e1;
  CHECK(e3.rows() == 0 && e3.data() == nullptr);
  CHECK(!(Matrix(1, 2) == Matrix(2, 1)));
  // dimension mismatches throw
  bool threw = false;
  try { Matrix x = Matrix(2, 2) + Matrix(2, 3); (void)x; } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw);
  threw = false;
  try { Matrix x = Matrix(2, 3) * Matrix(2, 3); (void)x; } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw);
  threw = false;
  Matrix keep{{1, 2}};
  try { keep += Matrix(1, 3); } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw && keep == (Matrix{{1, 2}}));
  // rvalue chain: A + B + C moves temporaries -> exactly one allocation (the copy of the first lvalue)
  Matrix A(6, 6, 1.0), B(6, 6, 2.0), C(6, 6, 3.0);
  before = Matrix::allocations();
  Matrix sum = A + B + C;
  CHECK_EQ(Matrix::allocations() - before, 1u);
  CHECK(sum == Matrix(6, 6, 6.0));
  // product correctness: (A B)^T == B^T A^T, I * X == X
  Matrix X{{1, -2, 0.5}, {3, 0, 4}}, Y{{2, 1}, {0, -1}, {5, 2}};
  CHECK((X * Y).transposed() == Y.transposed() * X.transposed());
  CHECK(Matrix::identity(2) * X == X);
  Matrix xy = X * Y;
  CHECK(dims(xy, 2, 2) && xy(0, 0) == 4.5 && xy(1, 1) == 11.0);
  // vector<Matrix> growth uses moves (noexcept) -> no extra allocations on reallocation
  std::vector<Matrix> v;
  before = Matrix::allocations();
  for (int i = 0; i < 20; ++i) v.emplace_back(2, 2, double(i));
  CHECK_EQ(Matrix::allocations() - before, 20u);
  CHECK(v[13] == Matrix(2, 2, 13.0));
#endif
  return TEST_SUMMARY();
}
