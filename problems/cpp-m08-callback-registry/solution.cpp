#include <cstddef>
#include <functional>
#include <map>
#include <vector>

struct ImuSample { double t; double ax, ay, az; };

class SensorBus {
 public:
  using Callback = std::function<void(const ImuSample&)>;

  int subscribe(Callback cb) {
    if (!cb) return -1;  // an empty std::function would throw bad_function_call on publish
    const int id = next_id_++;
    subs_.emplace(id, std::move(cb));
    return id;
  }

  template <class T>
  int subscribeMember(T* obj, void (T::*method)(const ImuSample&)) {
    if (!obj || !method) return -1;
    // The lambda stores the raw pointer: the bus does NOT extend obj's lifetime.
    return subscribe([obj, method](const ImuSample& s) { (obj->*method)(s); });
  }

  bool unsubscribe(int id) { return subs_.erase(id) > 0; }

  std::size_t publish(const ImuSample& s) {
    // Snapshot ids: callbacks may subscribe/unsubscribe while we iterate, which would
    // invalidate a live map iterator. New ids (added during publish) are not in the snapshot.
    std::vector<int> ids;
    ids.reserve(subs_.size());
    for (const auto& kv : subs_) ids.push_back(kv.first);

    std::size_t called = 0;
    for (int id : ids) {
      auto it = subs_.find(id);
      if (it == subs_.end()) continue;  // removed by an earlier callback in this publish
      // Invoke a COPY: if the callback unsubscribes itself, the stored std::function is
      // destroyed mid-call, but our copy (and its captures) stays alive until we return.
      Callback cb = it->second;
      cb(s);
      ++called;
    }
    return called;
  }

  std::size_t size() const { return subs_.size(); }

 private:
  std::map<int, Callback> subs_;  // ordered by id == subscription order
  int next_id_ = 1;
};
