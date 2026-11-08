#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

inline std::vector<std::string> g_log;

struct Tracer {
  std::string n;
  explicit Tracer(std::string s) : n(std::move(s)) { g_log.push_back("+" + n); }
  ~Tracer() { g_log.push_back("-" + n); }
};

class Sensor {
 public:
  virtual ~Sensor() {
    // TODO
  }
  const std::string& name() const { return name_; }
  int rateHz() const { return rate_hz_; }
  int frames() const { return frames_; }
  std::string describe() const {
    // TODO
    return "";
  }

 protected:
  Sensor(std::string name, int rate_hz) {
    // TODO
  }
  void tick() {
    // TODO
  }

 private:
  std::string name_;
  int rate_hz_ = 0;
  int frames_ = 0;
};

class Lidar : public Sensor {
 public:
  Lidar(std::string name, int rings, int rate_hz = 10) : Sensor(/* TODO */ "", 0) {
    // TODO
  }
  ~Lidar() override {
    // TODO
  }
  int rings() const { return rings_; }
  void spin(int revolutions) {
    // TODO
  }
  std::string describe() const {
    // TODO
    return "";
  }

 private:
  Tracer motor_{"motor"};
  int rings_ = 0;
};

class Camera : public Sensor {
 public:
  Camera(std::string name, int width, int height, int rate_hz = 30) : Sensor(/* TODO */ "", 0) {
    // TODO
  }
  ~Camera() override {
    // TODO
  }
  void capture() {
    // TODO
  }
  std::string describe() const {
    // TODO
    return "";
  }

 private:
  Tracer lens_{"lens"};
  Tracer shutter_{"shutter"};
  int w_ = 0, h_ = 0;
};
