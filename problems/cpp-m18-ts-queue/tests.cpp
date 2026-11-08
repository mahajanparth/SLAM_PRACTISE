// The candidate's code is injected before this file (g++ -include).
#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include "common/check.h"

using namespace std::chrono_literals;

int main() {
  // single-threaded FIFO
  ThreadSafeQueue<int> q;
  CHECK(q.push(1)); q.push(2); q.push(3);
  CHECK_EQ(q.size(), 3u);
  CHECK_EQ(q.tryPop().value_or(-1), 1);
  CHECK_EQ(q.pop().value_or(-1), 2);

  // a consumer blocked in pop() is woken by push()
  {
    ThreadSafeQueue<int> b;
    std::optional<int> got;
    std::thread consumer([&] { got = b.pop(); });
    std::this_thread::sleep_for(20ms);
    b.push(42);
    consumer.join();
    CHECK(got.has_value() && *got == 42);
  }

  // producers / consumers: every item delivered exactly once
  {
    ThreadSafeQueue<int> pc;
    std::atomic<long> sum{0};
    std::atomic<int> count{0};
    std::vector<std::thread> consumers, producers;
    for (int c = 0; c < 2; ++c)
      consumers.emplace_back([&] { while (auto v = pc.pop()) { sum += *v; ++count; } });
    for (int p = 0; p < 2; ++p)
      producers.emplace_back([&, p] { for (int i = 1; i <= 5000; ++i) pc.push(p * 100000 + i); });
    for (auto& t : producers) t.join();
    pc.close();
    for (auto& t : consumers) t.join();
    CHECK_EQ(count.load(), 10000);
    CHECK_EQ(sum.load(), 2L * (5000L * 5001 / 2) + 5000L * 100000);
  }

#ifdef HIDDEN
  // close() wakes ALL blocked consumers
  {
    ThreadSafeQueue<int> c;
    std::atomic<int> woke{0};
    std::vector<std::thread> waiters;
    for (int i = 0; i < 3; ++i) waiters.emplace_back([&] { if (!c.pop()) ++woke; });
    std::this_thread::sleep_for(20ms);
    c.close();
    for (auto& t : waiters) t.join();
    CHECK_EQ(woke.load(), 3);
    CHECK(c.closed());
  }
  // after close: push rejected, queued items drained, then nullopt
  {
    ThreadSafeQueue<int> d;
    d.push(7); d.push(8);
    d.close(); d.close();  // idempotent
    CHECK(!d.push(9));
    CHECK_EQ(d.pop().value_or(-1), 7);
    CHECK_EQ(d.pop().value_or(-1), 8);
    CHECK(!d.pop().has_value());
    CHECK(!d.tryPop().has_value());
  }
  // drop-oldest when bounded
  {
    ThreadSafeQueue<int> b(3);
    for (int i = 1; i <= 5; ++i) CHECK(b.push(i));
    CHECK_EQ(b.size(), 3u);
    CHECK_EQ(b.dropped(), 2u);
    CHECK_EQ(b.tryPop().value_or(-1), 3);
    CHECK_EQ(b.tryPop().value_or(-1), 4);
    CHECK_EQ(b.tryPop().value_or(-1), 5);
    CHECK(!b.tryPop().has_value());
  }
  // popFor: times out on empty (not early), returns an item pushed meanwhile
  {
    ThreadSafeQueue<int> t;
    auto t0 = std::chrono::steady_clock::now();
    auto none = t.popFor(15ms);
    auto waited = std::chrono::steady_clock::now() - t0;
    CHECK(!none.has_value());
    CHECK(waited >= 14ms);
    std::thread prod([&] { std::this_thread::sleep_for(10ms); t.push(5); });
    auto some = t.popFor(5000ms);
    prod.join();
    CHECK(some.has_value() && *some == 5);
  }
  // move-only payload, FIFO across many producers with a single consumer per-producer ordering
  {
    ThreadSafeQueue<std::unique_ptr<int>> u(2);
    u.push(std::make_unique<int>(1));
    u.push(std::make_unique<int>(2));
    u.push(std::make_unique<int>(3));  // drops 1 (freed, no leak)
    auto a = u.pop();
    CHECK(a && *a && **a == 2);
  }
  {
    ThreadSafeQueue<int> o;
    std::vector<int> seen;
    std::thread consumer([&] { while (auto v = o.pop()) seen.push_back(*v); });
    std::vector<std::thread> prods;
    for (int p = 0; p < 3; ++p)
      prods.emplace_back([&, p] { for (int i = 0; i < 2000; ++i) o.push(p * 10000 + i); });
    for (auto& t : prods) t.join();
    o.close();
    consumer.join();
    bool ordered = seen.size() == 6000;
    int last[3] = {-1, -1, -1};
    for (int v : seen) { int p = v / 10000, i = v % 10000; ordered = ordered && i > last[p]; last[p] = i; }
    CHECK(ordered);
  }
#endif
  return TEST_SUMMARY();
}
