#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

struct Point3 { double x, y, z; };

// ---------------- given: polymorphic descriptors ----------------
struct Descriptor {
  static inline int live = 0;
  static inline int clones = 0;
  Descriptor() { ++live; }
  Descriptor(const Descriptor&) { ++live; }
  Descriptor& operator=(const Descriptor&) = default;
  virtual ~Descriptor() { --live; }
  virtual std::unique_ptr<Descriptor> clone() const = 0;
  virtual std::string type() const = 0;
  virtual double score(const Descriptor& other) const = 0;
};

struct ScanContext : Descriptor {
  std::vector<float> rings;
  explicit ScanContext(std::vector<float> r) : rings(std::move(r)) {}
  std::unique_ptr<Descriptor> clone() const override { ++clones; return std::make_unique<ScanContext>(*this); }
  std::string type() const override { return "ScanContext"; }
  double score(const Descriptor& o) const override {
    auto* sc = dynamic_cast<const ScanContext*>(&o);
    if (!sc || sc->rings.size() != rings.size()) return 0.0;
    double s = 0;
    for (std::size_t i = 0; i < rings.size(); ++i) s += rings[i] * sc->rings[i];
    return s;
  }
};

struct M2DP : Descriptor {
  std::array<float, 192> sig{};
  std::unique_ptr<Descriptor> clone() const override { ++clones; return std::make_unique<M2DP>(*this); }
  std::string type() const override { return "M2DP"; }
  double score(const Descriptor& o) const override { return dynamic_cast<const M2DP*>(&o) ? 1.0 : 0.0; }
};
// -----------------------------------------------------------------

class Keyframe {
 public:
  Keyframe(int id, std::vector<Point3> cloud, std::unique_ptr<Descriptor> desc)
      : id_(id), cloud_(std::move(cloud)), desc_(std::move(desc)) {}

  Keyframe(const Keyframe& o) : id_(o.id_), cloud_(o.cloud_) {
    // TODO: the descriptor
  }
  Keyframe& operator=(const Keyframe& o) {
    // TODO
    return *this;
  }
  // TODO: what about the move operations?

  int id() const { return id_; }
  const std::vector<Point3>& cloud() const { return cloud_; }
  const Descriptor* descriptor() const { return desc_.get(); }
  void setDescriptor(std::unique_ptr<Descriptor> d) {
    // TODO
  }

  friend void swap(Keyframe& a, Keyframe& b) noexcept {
    // TODO
  }

 private:
  int id_;
  std::vector<Point3> cloud_;
  std::unique_ptr<Descriptor> desc_;
};
