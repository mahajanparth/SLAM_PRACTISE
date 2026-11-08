// Minimal CoderPad-style test macros. No gtest needed.
//   CHECK(cond);  CHECK_EQ(a,b);  CHECK_NEAR(a,b,tol);  CHECK_MAT_NEAR(A,B,tol);
//   Call TEST_SUMMARY() at the end of main (returns non-zero on failure).
#pragma once
#include <cmath>
#include <iostream>

namespace check_detail {
inline int& fails() { static int f = 0; return f; }
inline int& passes() { static int p = 0; return p; }
inline void report(bool ok, const char* expr, const char* file, int line) {
  if (ok) { ++passes(); return; }
  ++fails();
  std::cerr << "\033[31mFAIL\033[0m " << file << ":" << line << "  " << expr << "\n";
}
}  // namespace check_detail

#define CHECK(cond) check_detail::report(static_cast<bool>(cond), #cond, __FILE__, __LINE__)

#define CHECK_EQ(a, b)                                                              \
  do {                                                                              \
    auto _va = (a); auto _vb = (b);                                                 \
    bool _ok = (_va == _vb);                                                        \
    check_detail::report(_ok, #a " == " #b, __FILE__, __LINE__);                    \
    if (!_ok) std::cerr << "     got " << _va << " vs " << _vb << "\n";            \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                       \
  do {                                                                              \
    double _va = (a), _vb = (b);                                                    \
    bool _ok = std::abs(_va - _vb) <= (tol);                                        \
    check_detail::report(_ok, #a " ~= " #b, __FILE__, __LINE__);                    \
    if (!_ok) std::cerr << "     got " << _va << " vs " << _vb << "\n";            \
  } while (0)

// Works for any Eigen dense expressions of equal size.
#define CHECK_MAT_NEAR(A, B, tol)                                                   \
  do {                                                                              \
    const auto _A = (A).eval(); const auto _B = (B).eval();  /* copy: no dangling */ \
    bool _ok = _A.rows() == _B.rows() && _A.cols() == _B.cols() &&                  \
               ((_A - _B).cwiseAbs().maxCoeff() <= (tol));                          \
    check_detail::report(_ok, #A " ~= " #B, __FILE__, __LINE__);                    \
    if (!_ok) std::cerr << "     A=\n" << _A << "\n     B=\n" << _B << "\n";       \
  } while (0)

#define TEST_SUMMARY()                                                              \
  (std::cout << (check_detail::fails() ? "\033[31m" : "\033[32m")                  \
             << check_detail::passes() << " passed, " << check_detail::fails()      \
             << " failed\033[0m\n",                                                 \
   check_detail::fails() ? 1 : 0)
