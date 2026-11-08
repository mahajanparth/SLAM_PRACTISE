#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>
#include <utility>

class Matrix {
 public:
  Matrix() = default;
  Matrix(std::size_t rows, std::size_t cols, double fill = 0.0)
      : rows_(rows), cols_(cols), data_(allocate(rows * cols)) {
    std::fill(data_, data_ + rows * cols, fill);
  }
  Matrix(std::initializer_list<std::initializer_list<double>> init)
      : rows_(init.size()), cols_(init.size() ? init.begin()->size() : 0) {
    for (const auto& row : init)
      if (row.size() != cols_) throw std::invalid_argument("ragged initializer");
    data_ = allocate(rows_ * cols_);
    std::size_t i = 0;
    for (const auto& row : init)
      for (double v : row) data_[i++] = v;
  }
  ~Matrix() { delete[] data_; }

  // Deep copy.
  Matrix(const Matrix& o) : rows_(o.rows_), cols_(o.cols_), data_(allocate(o.size())) {
    std::copy(o.data_, o.data_ + o.size(), data_);
  }
  // Copy-and-swap: allocate first (may throw, *this untouched), then a non-throwing swap.
  // Self-assignment is correct (just a wasted copy).
  Matrix& operator=(const Matrix& o) {
    Matrix tmp(o);
    swap(tmp);
    return *this;
  }
  // Move: steal the buffer, leave the source a valid empty matrix. No allocation, cannot throw.
  Matrix(Matrix&& o) noexcept
      : rows_(std::exchange(o.rows_, 0)), cols_(std::exchange(o.cols_, 0)), data_(std::exchange(o.data_, nullptr)) {}
  Matrix& operator=(Matrix&& o) noexcept {
    if (this != &o) {
      delete[] data_;
      rows_ = std::exchange(o.rows_, 0);
      cols_ = std::exchange(o.cols_, 0);
      data_ = std::exchange(o.data_, nullptr);
    }
    return *this;
  }

  void swap(Matrix& o) noexcept {
    std::swap(rows_, o.rows_);
    std::swap(cols_, o.cols_);
    std::swap(data_, o.data_);
  }

  std::size_t rows() const { return rows_; }
  std::size_t cols() const { return cols_; }
  const double* data() const { return data_; }
  double& operator()(std::size_t r, std::size_t c) { return data_[r * cols_ + c]; }
  double operator()(std::size_t r, std::size_t c) const { return data_[r * cols_ + c]; }

  Matrix& operator+=(const Matrix& o) {
    if (rows_ != o.rows_ || cols_ != o.cols_) throw std::invalid_argument("dimension mismatch in +=");
    for (std::size_t i = 0; i < size(); ++i) data_[i] += o.data_[i];
    return *this;
  }
  Matrix transposed() const {
    Matrix t(cols_, rows_);
    for (std::size_t r = 0; r < rows_; ++r)
      for (std::size_t c = 0; c < cols_; ++c) t(c, r) = (*this)(r, c);
    return t;  // NRVO / move, no extra allocation
  }
  bool operator==(const Matrix& o) const {
    return rows_ == o.rows_ && cols_ == o.cols_ && std::equal(data_, data_ + size(), o.data_);
  }
  static Matrix identity(std::size_t n) {
    Matrix m(n, n, 0.0);
    for (std::size_t i = 0; i < n; ++i) m(i, i) = 1.0;
    return m;
  }

  static std::size_t& allocations() { static std::size_t n = 0; return n; }

 private:
  static double* allocate(std::size_t n) {
    if (n == 0) return nullptr;
    ++allocations();
    return new double[n];
  }
  std::size_t size() const { return rows_ * cols_; }

  std::size_t rows_ = 0, cols_ = 0;
  double* data_ = nullptr;
};

// `a` by value: an lvalue argument is copied once (we need a result buffer anyway),
// an rvalue argument (e.g. A + B + C) is moved in, so no allocation at all.
Matrix operator+(Matrix a, const Matrix& b) {
  a += b;
  return a;
}

Matrix operator*(const Matrix& a, const Matrix& b) {
  if (a.cols() != b.rows()) throw std::invalid_argument("dimension mismatch in *");
  Matrix c(a.rows(), b.cols(), 0.0);
  for (std::size_t i = 0; i < a.rows(); ++i)
    for (std::size_t k = 0; k < a.cols(); ++k) {
      const double aik = a(i, k);
      for (std::size_t j = 0; j < b.cols(); ++j) c(i, j) += aik * b(k, j);  // i-k-j: contiguous rows of b and c
    }
  return c;
}
