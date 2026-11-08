#include <string>
#include <utility>
#include <vector>

inline std::vector<std::string> g_log;

class Device {
 public:
  explicit Device(int id) : id_(id) { g_log.push_back("Device(" + std::to_string(id_) + ")"); }
  virtual ~Device() { g_log.push_back("~Device(" + std::to_string(id_) + ")"); }
  int id() const { return id_; }
  int bootCount() const { return boot_count_; }

 protected:
  void boot() { ++boot_count_; }

 private:
  int id_;
  int boot_count_ = 0;
};

// `virtual` base: all classes deriving virtually from Device share ONE Device subobject
// in the most-derived object.
class Powered : public virtual Device {
 public:
  // Used for a standalone Powered. When Powered is a base of SmartLidar, this Device(id)
  // call is IGNORED: the most-derived class initialises the virtual base.
  Powered(int id, double watts) : Device(id), watts_(watts) {
    g_log.push_back("Powered(" + std::to_string(this->id()) + ")");
  }
  ~Powered() override { g_log.push_back("~Powered(" + std::to_string(id()) + ")"); }
  double watts() const { return watts_; }
  bool isOn() const { return on_; }
  void powerOn() {
    if (on_) return;
    on_ = true;
    boot();  // protected member of the (shared) Device
  }

 private:
  double watts_;
  bool on_ = false;
};

class Networked : public virtual Device {
 public:
  Networked(int id, std::string ip) : Device(id), ip_(std::move(ip)) {
    g_log.push_back("Networked(" + std::to_string(this->id()) + ")");
  }
  ~Networked() override { g_log.push_back("~Networked(" + std::to_string(id()) + ")"); }
  const std::string& ip() const { return ip_; }
  void connect() { boot(); }

 private:
  std::string ip_;
};

class SmartLidar : public Powered, public Networked {
 public:
  // Construction order: virtual base Device FIRST, then Powered, then Networked (declaration
  // order of the bases, not the order written here), then this body.
  SmartLidar(int id, double watts, std::string ip)
      : Device(id), Powered(id, watts), Networked(id, std::move(ip)) {
    g_log.push_back("SmartLidar(" + std::to_string(this->id()) + ")");
  }
  ~SmartLidar() override { g_log.push_back("~SmartLidar(" + std::to_string(id()) + ")"); }
  // Unambiguous now: there is exactly one Device::id / Device::bootCount.
  int deviceId() const { return id(); }
  int totalBoots() const { return bootCount(); }
};
