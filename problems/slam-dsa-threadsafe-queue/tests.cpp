// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <atomic>
#include <memory>
#include <string>
#include <vector>

int main() {
  // single-threaded FIFO
  ThreadSafeQueue<int> q;
  CHECK(q.push(1));
  CHECK(q.push(2));
  CHECK_EQ(q.size(), std::size_t(2));
  auto a = q.pop();
  CHECK(a.has_value() && *a == 1);
  auto b = q.tryPop();
  CHECK(b.has_value() && *b == 2);
  CHECK(!q.tryPop().has_value());

  // producer / consumer
  {
    ThreadSafeQueue<int> pq;
    long long sum = 0;
    int count = 0;
    std::thread consumer([&] {
      while (auto v = pq.pop()) { sum += *v; ++count; }
    });
    std::thread producer([&] { for (int i = 1; i <= 1000; ++i) pq.push(i); });
    producer.join();
    pq.shutdown();  // items already queued must still be drained
    consumer.join();
    CHECK_EQ(count, 1000);
    CHECK_EQ(sum, 500500LL);
  }

#ifdef HIDDEN
  // shutdown wakes a blocked consumer (no hang) and rejects further pushes
  {
    ThreadSafeQueue<int> sq;
    std::atomic<bool> got_null{false};
    std::thread c([&] { got_null = !sq.pop().has_value(); });
    sq.shutdown();
    c.join();
    CHECK(got_null.load());
    CHECK(!sq.push(5));
    CHECK_EQ(sq.size(), std::size_t(0));
  }
  // many producers, many consumers
  {
    ThreadSafeQueue<int> mq;
    std::atomic<long long> sum{0};
    std::atomic<int> count{0};
    std::vector<std::thread> cons, prods;
    for (int k = 0; k < 3; ++k)
      cons.emplace_back([&] { while (auto v = mq.pop()) { sum += *v; ++count; } });
    for (int k = 0; k < 4; ++k)
      prods.emplace_back([&, k] { for (int i = 0; i < 500; ++i) mq.push(k * 500 + i + 1); });
    for (auto& t : prods) t.join();
    mq.shutdown();
    for (auto& t : cons) t.join();
    CHECK_EQ(count.load(), 2000);
    CHECK_EQ(sum.load(), 2001000LL);
  }
  // bounded: drop oldest
  {
    ThreadSafeQueue<int> bq(3);
    for (int i = 1; i <= 5; ++i) bq.push(i);
    CHECK_EQ(bq.size(), std::size_t(3));
    auto f = bq.tryPop();
    CHECK(f.has_value() && *f == 3);
  }
  // move-only payload (a scan owned by unique_ptr)
  {
    ThreadSafeQueue<std::unique_ptr<std::string>> uq;
    uq.push(std::make_unique<std::string>("scan"));
    auto s = uq.pop();
    CHECK(s.has_value() && *s && **s == "scan");
  }
#endif
  return TEST_SUMMARY();
}
