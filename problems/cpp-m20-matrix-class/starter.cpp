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

  Matrix(const Matrix& o) {
    // TODO: deep copy
  }
  Matrix& operator=(const Matrix& o) {
    // TODO
    return *this;
  }
  Matrix(Matrix&& o) noexcept {
    // TODO: steal o's buffer, leave o empty
  }
  Matrix& operator=(Matrix&& o) noexcept {
    // TODO
    return *this;
  }

  std::size_t rows() const { return rows_; }
  std::size_t cols() const { return cols_; }
  const double* data() const { return data_; }
  double& operator()(std::size_t r, std::size_t c) { return data_[r * cols_ + c]; }
  double operator()(std::size_t r, std::size_t c) const { return data_[r * cols_ + c]; }

  Matrix& operator+=(const Matrix& o) {
    // TODO
    return *this;
  }
  Matrix transposed() const {
    // TODO
    return {};
  }
  bool operator==(const Matrix& o) const {
    // TODO
    return false;
  }
  static Matrix identity(std::size_t n) {
    // TODO
    return {};
  }

  static std::size_t& allocations() { static std::size_t n = 0; return n; }

 private:
  static double* allocate(std::size_t n) {
    if (n == 0) return nullptr;
    ++allocations();
    return new double[n];
  }

  std::size_t rows_ = 0, cols_ = 0;
  double* data_ = nullptr;
};

Matrix operator+(Matrix a, const Matrix& b) {
  // TODO
  return a;
}

Matrix operator*(const Matrix& a, const Matrix& b) {
  // TODO
  return {};
}
