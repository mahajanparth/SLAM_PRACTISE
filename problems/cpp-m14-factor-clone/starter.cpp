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
  double error(const std::vector<double>& x) const override {
    // TODO
    return 0.0;
  }
  std::unique_ptr<Factor> clone() const override {
    // TODO
    return nullptr;
  }
  std::string type() const override { return ""; /* TODO */ }

 private:
  int key_;
  double mean_, sigma_;
};

class BetweenFactor : public Factor {
 public:
  BetweenFactor(int i, int j, double meas, double sigma) : i_(i), j_(j), meas_(meas), sigma_(sigma) {}
  double error(const std::vector<double>& x) const override {
    // TODO
    return 0.0;
  }
  std::unique_ptr<Factor> clone() const override {
    // TODO
    return nullptr;
  }
  std::string type() const override { return ""; /* TODO */ }

 private:
  int i_, j_;
  double meas_, sigma_;
};

class FactorGraph {
 public:
  FactorGraph() = default;
  FactorGraph(const FactorGraph& other) {
    // TODO: deep copy
  }
  FactorGraph& operator=(const FactorGraph& other) {
    // TODO: deep copy, self-assignment safe
    return *this;
  }
  FactorGraph(FactorGraph&&) noexcept = default;
  FactorGraph& operator=(FactorGraph&&) noexcept = default;
  ~FactorGraph() = default;

  void add(std::unique_ptr<Factor> f) { if (f) factors_.push_back(std::move(f)); }
  std::size_t size() const { return factors_.size(); }
  const Factor& at(std::size_t i) const { return *factors_.at(i); }

  double totalError(const std::vector<double>& x) const {
    // TODO: 0.5 * sum of squared whitened residuals
    return -1.0;
  }
  std::map<std::string, int> countByType() const {
    // TODO
    return {};
  }

 private:
  std::vector<std::unique_ptr<Factor>> factors_;
};

std::unique_ptr<Factor> parseFactor(const std::string& line) {
  // TODO: "prior k mean sigma" | "between i j meas sigma"; nullptr if malformed
  return nullptr;
}
