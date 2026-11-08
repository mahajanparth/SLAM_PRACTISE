#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class Filter {
 public:
  virtual ~Filter() = default;  // deleting a derived filter through Filter* must run the derived dtor
  virtual std::vector<double> apply(const std::vector<double>& in) const = 0;
  virtual std::string name() const = 0;
};

class ClampFilter : public Filter {
 public:
  ClampFilter(double lo, double hi) : lo_(lo), hi_(hi) {}
  std::vector<double> apply(const std::vector<double>& in) const override {
    std::vector<double> out;
    out.reserve(in.size());
    for (double v : in) out.push_back(std::clamp(v, lo_, hi_));
    return out;
  }
  std::string name() const override { return "clamp"; }

 private:
  double lo_, hi_;
};

class ScaleFilter : public Filter {
 public:
  explicit ScaleFilter(double s) : s_(s) {}
  std::vector<double> apply(const std::vector<double>& in) const override {
    std::vector<double> out(in);
    for (double& v : out) v *= s_;
    return out;
  }
  std::string name() const override { return "scale"; }

 private:
  double s_;
};

class MovingAverage : public Filter {
 public:
  explicit MovingAverage(std::size_t window) : window_(window) {}
  std::vector<double> apply(const std::vector<double>& in) const override {
    std::vector<double> out(in.size());
    double sum = 0.0;  // running sum over the last `window_` values -> O(n)
    for (std::size_t i = 0; i < in.size(); ++i) {
      sum += in[i];
      if (i >= window_) sum -= in[i - window_];
      const std::size_t count = std::min(i + 1, window_);
      out[i] = sum / static_cast<double>(count);
    }
    return out;
  }
  std::string name() const override { return "movavg"; }

 private:
  std::size_t window_;
};

class FilterChain {
 public:
  void add(std::unique_ptr<Filter> f) {
    if (f) filters_.push_back(std::move(f));  // unique_ptr is move-only: ownership transfers here
  }
  std::vector<double> apply(const std::vector<double>& in) const {
    std::vector<double> out = in;
    for (const auto& f : filters_) out = f->apply(out);  // one virtual call per filter per scan
    return out;
  }
  std::string describe() const {
    std::string s;
    for (const auto& f : filters_) {
      if (!s.empty()) s += "->";
      s += f->name();
    }
    return s;
  }
  std::size_t size() const { return filters_.size(); }

 private:
  std::vector<std::unique_ptr<Filter>> filters_;
};
