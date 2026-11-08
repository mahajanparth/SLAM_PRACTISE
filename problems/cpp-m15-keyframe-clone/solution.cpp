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

  // unique_ptr is move-only, so the implicit copy ctor is deleted: we write a deep copy.
  // clone() is virtual, so the copy keeps the dynamic type (no slicing).
  Keyframe(const Keyframe& o)
      : id_(o.id_), cloud_(o.cloud_), desc_(o.desc_ ? o.desc_->clone() : nullptr) {}

  // Copy-and-swap: strong guarantee (if clone()/vector copy throws, *this is untouched)
  // and self-assignment safe.
  Keyframe& operator=(const Keyframe& o) {
    Keyframe tmp(o);
    swap(*this, tmp);
    return *this;
  }

  // Declaring the copy operations suppressed the implicit moves; bring them back.
  // Member-wise move of int / vector / unique_ptr never throws -> noexcept, so
  // std::vector<Keyframe> moves (not copies) on reallocation.
  Keyframe(Keyframe&&) noexcept = default;
  Keyframe& operator=(Keyframe&&) noexcept = default;
  // No destructor: the members clean up after themselves.

  int id() const { return id_; }
  const std::vector<Point3>& cloud() const { return cloud_; }
  const Descriptor* descriptor() const { return desc_.get(); }
  void setDescriptor(std::unique_ptr<Descriptor> d) { desc_ = std::move(d); }

  friend void swap(Keyframe& a, Keyframe& b) noexcept {
    using std::swap;
    swap(a.id_, b.id_);
    swap(a.cloud_, b.cloud_);  // O(1): swaps buffer pointers
    swap(a.desc_, b.desc_);
  }

 private:
  int id_;
  std::vector<Point3> cloud_;
  std::unique_ptr<Descriptor> desc_;
};
