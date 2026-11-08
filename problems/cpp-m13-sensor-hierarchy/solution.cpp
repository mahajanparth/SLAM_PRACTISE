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
  // Virtual: deleting a Lidar through a Sensor* (e.g. unique_ptr<Sensor>) must run ~Lidar.
  virtual ~Sensor() { g_log.push_back("~Sensor(" + name_ + ")"); }
  const std::string& name() const { return name_; }
  int rateHz() const { return rate_hz_; }
  int frames() const { return frames_; }
  std::string describe() const { return name_ + "@" + std::to_string(rate_hz_) + "Hz"; }

 protected:
  // Protected: Sensor is only meaningful as a base; users can't create a bare Sensor.
  Sensor(std::string name, int rate_hz) : name_(std::move(name)), rate_hz_(rate_hz) {
    g_log.push_back("Sensor(" + name_ + ")");
  }
  void tick() { ++frames_; }  // interface for implementers, not for users

 private:
  std::string name_;  // private even to derived classes: they go through name()/tick()
  int rate_hz_ = 0;
  int frames_ = 0;
};

class Lidar : public Sensor {
 public:
  // Order: Sensor base -> motor_ -> rings_ -> body (independent of how the list is written).
  Lidar(std::string name, int rings, int rate_hz = 10)
      : Sensor(std::move(name), rate_hz), rings_(rings) {
    // Throwing here: motor_ and the Sensor base are already constructed, so they ARE
    // destroyed during unwinding, but ~Lidar() is NOT called (the object never finished).
    if (rings < 1) throw std::invalid_argument("rings must be >= 1");
    g_log.push_back("Lidar(" + this->name() + ")");
  }
  ~Lidar() override { g_log.push_back("~Lidar(" + name() + ")"); }  // then -motor, then ~Sensor
  int rings() const { return rings_; }
  void spin(int revolutions) {
    for (int i = 0; i < revolutions; ++i) tick();  // protected base member: OK in derived
  }
  // Hides (does not override) Sensor::describe; call the base version explicitly.
  std::string describe() const { return Sensor::describe() + " rings=" + std::to_string(rings_); }

 private:
  Tracer motor_{"motor"};
  int rings_ = 0;
};

class Camera : public Sensor {
 public:
  Camera(std::string name, int width, int height, int rate_hz = 30)
      : Sensor(std::move(name), rate_hz), w_(width), h_(height) {
    g_log.push_back("Camera(" + this->name() + ")");
  }
  ~Camera() override { g_log.push_back("~Camera(" + name() + ")"); }  // then -shutter, -lens
  void capture() { tick(); }
  std::string describe() const {
    return Sensor::describe() + " " + std::to_string(w_) + "x" + std::to_string(h_);
  }

 private:
  Tracer lens_{"lens"};       // constructed first, destroyed last
  Tracer shutter_{"shutter"};
  int w_ = 0, h_ = 0;
};
