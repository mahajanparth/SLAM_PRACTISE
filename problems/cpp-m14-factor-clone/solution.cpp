#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

class Factor {
 public:
  virtual ~Factor() = default;
  virtual double error(const std::vector<double>& x) const = 0;
  virtual std::unique_ptr<Factor> clone() const = 0;
  virtual std::string type() const = 0;
};

class PriorFactor : public Factor {
 public:
  PriorFactor(int key, double mean, double sigma) : key_(key), mean_(mean), sigma_(sigma) {}
  double error(const std::vector<double>& x) const override { return (x[key_] - mean_) / sigma_; }
  // Only the most-derived class knows its own type, so it does the copy.
  std::unique_ptr<Factor> clone() const override { return std::make_unique<PriorFactor>(*this); }
  std::string type() const override { return "prior"; }

 private:
  int key_;
  double mean_, sigma_;
};

class BetweenFactor : public Factor {
 public:
  BetweenFactor(int i, int j, double meas, double sigma) : i_(i), j_(j), meas_(meas), sigma_(sigma) {}
  double error(const std::vector<double>& x) const override { return (x[j_] - x[i_] - meas_) / sigma_; }
  std::unique_ptr<Factor> clone() const override { return std::make_unique<BetweenFactor>(*this); }
  std::string type() const override { return "between"; }

 private:
  int i_, j_;
  double meas_, sigma_;
};

class FactorGraph {
 public:
  FactorGraph() = default;
  FactorGraph(const FactorGraph& other) {
    factors_.reserve(other.factors_.size());
    for (const auto& f : other.factors_) factors_.push_back(f->clone());
  }
  // Copy-and-swap: if a clone throws, *this is untouched (strong guarantee); self-assignment just works.
  FactorGraph& operator=(const FactorGraph& other) {
    FactorGraph tmp(other);
    std::swap(factors_, tmp.factors_);
    return *this;
  }
  // A user-declared copy ctor suppresses the implicit moves, so we default them explicitly.
  FactorGraph(FactorGraph&&) noexcept = default;
  FactorGraph& operator=(FactorGraph&&) noexcept = default;
  ~FactorGraph() = default;

  void add(std::unique_ptr<Factor> f) { if (f) factors_.push_back(std::move(f)); }
  std::size_t size() const { return factors_.size(); }
  const Factor& at(std::size_t i) const { return *factors_.at(i); }

  double totalError(const std::vector<double>& x) const {
    double sum = 0.0;
    for (const auto& f : factors_) {
      const double r = f->error(x);
      sum += r * r;
    }
    return 0.5 * sum;
  }
  std::map<std::string, int> countByType() const {
    std::map<std::string, int> counts;
    for (const auto& f : factors_) ++counts[f->type()];
    return counts;
  }

 private:
  std::vector<std::unique_ptr<Factor>> factors_;
};

std::unique_ptr<Factor> parseFactor(const std::string& line) {
  std::istringstream iss(line);
  std::string kind;
  if (!(iss >> kind)) return nullptr;
  std::unique_ptr<Factor> f;
  if (kind == "prior") {
    int k; double mean, sigma;
    if (!(iss >> k >> mean >> sigma) || k < 0 || !(sigma > 0)) return nullptr;
    f = std::make_unique<PriorFactor>(k, mean, sigma);
  } else if (kind == "between") {
    int i, j; double meas, sigma;
    if (!(iss >> i >> j >> meas >> sigma) || i < 0 || j < 0 || !(sigma > 0)) return nullptr;
    f = std::make_unique<BetweenFactor>(i, j, meas, sigma);
  } else {
    return nullptr;
  }
  std::string extra;
  if (iss >> extra) return nullptr;  // trailing garbage
  return f;  // unique_ptr<Factor> returned by move; no leak on any early return
}
