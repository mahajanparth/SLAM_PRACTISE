#include <cstddef>
#include <functional>
#include <map>
#include <vector>

struct ImuSample { double t; double ax, ay, az; };

class SensorBus {
 public:
  using Callback = std::function<void(const ImuSample&)>;

  int subscribe(Callback cb) {
    // TODO
    return -1;
  }

  template <class T>
  int subscribeMember(T* obj, void (T::*method)(const ImuSample&)) {
    // TODO
    return -1;
  }

  bool unsubscribe(int id) {
    // TODO
    return false;
  }

  std::size_t publish(const ImuSample& s) {
    // TODO
    return 0;
  }

  std::size_t size() const {
    // TODO
    return 0;
  }

 private:
  // Suggested storage (change it if you like).
  std::map<int, Callback> subs_;
  int next_id_ = 1;
};
