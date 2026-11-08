// Module 1 — Fundamentals: references, pointers, const, auto
// Run: ./run.sh scratch/cpp/m01_fundamentals.cpp
#include <cstdio>
#include <vector>

struct Point3 { double x, y, z; };

int main() {
  std::vector<Point3> cloud = {{1, 2, 3}, {4, 5, 6}};

  // --- 1. auto deduction: drops top-level const and references ---
  for (auto p : cloud) p.x = 100;          // p is a COPY -> cloud unchanged
  std::printf("after auto  : %.0f\n", cloud[0].x);
  for (auto& p : cloud) p.x = 100;         // p is a REFERENCE -> modifies cloud
  std::printf("after auto& : %.0f\n", cloud[0].x);
  for (const auto& p : cloud) std::printf("read-only  : %.0f\n", p.y);  // no copy, no modify

  // --- 2. reference vs pointer ---
  Point3& r = cloud[0];   // must bind at init, can't be reseated, never null
  Point3* ptr = &cloud[1]; // can be null, can be reseated, needs ->
  r.z = -1;
  ptr->z = -2;
  ptr = &cloud[0];        // reseat: now points to element 0
  std::printf("r.z=%.0f ptr->z=%.0f\n", r.z, ptr->z);

  // --- 3. const placement: read right-to-left ---
  const Point3* p1 = &cloud[0];       // pointer to const Point3: can't modify *p1, can reseat p1
  Point3* const p2 = &cloud[0];       // const pointer to Point3: can modify *p2, can't reseat
  // p1->x = 0;   // error
  // p2 = &cloud[1]; // error
  p2->x = 7;
  (void)p1;

  // --- 4. size_t underflow (classic bug) ---
  std::vector<int> empty;
  std::printf("empty.size()-1 = %zu\n", empty.size() - 1);  // huge number, not -1!
  // for (size_t i = 0; i < empty.size() - 1; ++i)  -> loops ~forever / out of bounds

  // --- 5. brace init prevents narrowing ---
  int a = 3.7;      // silently truncates to 3 (warning at best)
  // int b{3.7};    // compile ERROR: narrowing
  int c{};          // value-initialised to 0; `int d;` would be garbage
  std::printf("a=%d c=%d\n", a, c);
  return 0;
}
