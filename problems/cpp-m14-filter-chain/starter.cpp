#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class Filter {
 public:
  virtual ~Filter() = default;
  virtual std::vector<double> apply(const std::vector<double>& in) const = 0;
  virtual std::string name() const = 0;
};

class ClampFilter : public Filter {
 public:
  ClampFilter(double lo, double hi) : lo_(lo), hi_(hi) {}
  std::vector<double> apply(const std::vector<double>& in) const override {
    // TODO
    return {};
  }
  std::string name() const override { return ""; /* TODO */ }

 private:
  double lo_, hi_;
};

class ScaleFilter : public Filter {
 public:
  explicit ScaleFilter(double s) : s_(s) {}
  std::vector<double> apply(const std::vector<double>& in) const override {
    // TODO
    return {};
  }
  std::string name() const override { return ""; /* TODO */ }

 private:
  double s_;
};

class MovingAverage : public Filter {
 public:
  explicit MovingAverage(std::size_t window) : window_(window) {}
  std::vector<double> apply(const std::vector<double>& in) const override {
    // TODO: causal moving average, O(n)
    return {};
  }
  std::string name() const override { return ""; /* TODO */ }

 private:
  std::size_t window_;
};

class FilterChain {
 public:
  void add(std::unique_ptr<Filter> f) {
    // TODO
  }
  std::vector<double> apply(const std::vector<double>& in) const {
    // TODO
    return {};
  }
  std::string describe() const {
    // TODO
    return "?";
  }
  std::size_t size() const { return filters_.size(); }

 private:
  std::vector<std::unique_ptr<Filter>> filters_;
};
