// The candidate's code is injected before this file (g++ -include).
#include <type_traits>
#include "common/check.h"

static Keyframe makeKf(int id, std::size_t n) {
  std::vector<Point3> cloud(n, Point3{double(id), 0, 0});
  return Keyframe(id, std::move(cloud), std::make_unique<ScanContext>(std::vector<float>{1.f, 2.f, 3.f}));
}

int main() {
  {
    Keyframe a = makeKf(1, 100);
    Descriptor::clones = 0;
    Keyframe b = a;  // deep copy
    CHECK(b.descriptor() != nullptr && b.descriptor() != a.descriptor());
    CHECK(b.descriptor() && b.descriptor()->type() == "ScanContext");
    CHECK_EQ(Descriptor::clones, 1);

    const Point3* buf = a.cloud().data();
    const Descriptor* d = a.descriptor();
    Keyframe c = std::move(a);  // must not clone or copy the cloud
    CHECK(c.cloud().data() == buf && c.descriptor() == d);
    CHECK_EQ(Descriptor::clones, 1);

    Descriptor::clones = 0;
    std::vector<Keyframe> db;
    for (int i = 0; i < 16; ++i) db.push_back(makeKf(i, 10));
    CHECK_EQ(Descriptor::clones, 0);
  }
  CHECK_EQ(Descriptor::live, 0);

#ifdef HIDDEN
  CHECK(std::is_nothrow_move_constructible_v<Keyframe> && std::is_nothrow_move_assignable_v<Keyframe>);
  {
    // dynamic type preserved; null descriptor stays null
    Keyframe m(7, {}, std::make_unique<M2DP>());
    Keyframe m2 = m;
    CHECK(m2.descriptor() && m2.descriptor()->type() == "M2DP" && m2.id() == 7);
    Keyframe n(8, {{1, 2, 3}}, nullptr);
    Descriptor::clones = 0;
    Keyframe n2 = n;
    CHECK(n2.descriptor() == nullptr && n2.cloud().size() == 1 && Descriptor::clones == 0);

    // copy assignment: replaces old descriptor (no leak), self-assign safe
    Keyframe a = makeKf(1, 5);
    a = m;
    CHECK(a.id() == 7 && a.descriptor() && a.descriptor() != m.descriptor() && a.descriptor()->type() == "M2DP");
    Keyframe& ar = a;
    const int liveBefore = Descriptor::live;
    a = ar;
    CHECK(a.id() == 7 && a.descriptor() && a.descriptor()->type() == "M2DP" && Descriptor::live == liveBefore);
    a = n;
    CHECK(a.descriptor() == nullptr && a.cloud().size() == 1);

    // move assignment transfers, frees the old descriptor
    Keyframe t = makeKf(3, 50);
    const Descriptor* td = t.descriptor();
    const int live0 = Descriptor::live;
    Descriptor::clones = 0;
    m = std::move(t);
    CHECK(m.descriptor() == td && m.id() == 3 && Descriptor::clones == 0 && Descriptor::live == live0 - 1);

    // swap: no clone, contents exchanged
    Keyframe s1 = makeKf(10, 3), s2(11, {}, std::make_unique<M2DP>());
    Descriptor::clones = 0;
    swap(s1, s2);
    CHECK(s1.id() == 11 && s2.id() == 10 && s1.descriptor() && s1.descriptor()->type() == "M2DP" &&
          s2.cloud().size() == 3 && Descriptor::clones == 0);

    // setDescriptor replaces
    const int live1 = Descriptor::live;
    s1.setDescriptor(std::make_unique<ScanContext>(std::vector<float>{2.f}));
    CHECK(s1.descriptor() && s1.descriptor()->type() == "ScanContext" && Descriptor::live == live1);
    s1.setDescriptor(nullptr);
    CHECK(s1.descriptor() == nullptr && Descriptor::live == live1 - 1);

    // vector copy clones each; erase shifts by move (no clones)
    std::vector<Keyframe> v;
    for (int i = 0; i < 8; ++i) v.push_back(makeKf(i, 4));
    Descriptor::clones = 0;
    std::vector<Keyframe> w = v;
    CHECK_EQ(Descriptor::clones, 8);
    Descriptor::clones = 0;
    w.erase(w.begin());
    CHECK(Descriptor::clones == 0 && w.front().id() == 1);
    CHECK(w[0].descriptor() && v[1].descriptor() && w[0].descriptor()->score(*v[1].descriptor()) == 14.0);
  }
  CHECK_EQ(Descriptor::live, 0);
#endif
  return TEST_SUMMARY();
}
