#include <string>
#include <utility>
#include <vector>

inline std::vector<std::string> g_log;

class Device {
 public:
  explicit Device(int id) : id_(id) {
    // TODO
  }
  virtual ~Device() {
    // TODO
  }
  int id() const { return id_; }
  int bootCount() const { return boot_count_; }

 protected:
  void boot() {
    // TODO
  }

 private:
  int id_;
  int boot_count_ = 0;
};

// TODO: a SmartLidar must contain exactly ONE Device.
class Powered : public Device {
 public:
  Powered(int id, double watts) : Device(id), watts_(watts) {
    // TODO
  }
  ~Powered() override {
    // TODO
  }
  double watts() const { return watts_; }
  bool isOn() const { return on_; }
  void powerOn() {
    // TODO
  }

 private:
  double watts_;
  bool on_ = false;
};

class Networked : public Device {
 public:
  Networked(int id, std::string ip) : Device(id), ip_(std::move(ip)) {
    // TODO
  }
  ~Networked() override {
    // TODO
  }
  const std::string& ip() const { return ip_; }
  void connect() {
    // TODO
  }

 private:
  std::string ip_;
};

class SmartLidar : public Powered, public Networked {
 public:
  SmartLidar(int id, double watts, std::string ip) : Powered(id, watts), Networked(id, std::move(ip)) {
    // TODO
  }
  ~SmartLidar() override {
    // TODO
  }
  int deviceId() const {
    // TODO
    return -1;
  }
  int totalBoots() const {
    // TODO
    return -1;
  }
};
