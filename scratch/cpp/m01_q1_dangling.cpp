// Module 1, Q1 — reference invalidated by vector reallocation
#include <iostream>
#include <vector>
struct Point3 { double x, y, z; };
int main() {
  std::vector<Point3> cloud = {{1, 2, 3}};
  std::cout << "capacity before: " << cloud.capacity() << "\n";
  Point3& first = cloud[0];
  cloud.push_back({4, 5, 6});            // capacity 1 -> reallocates, old buffer freed
  std::cout << "capacity after : " << cloud.capacity() << "\n";
  first.x = 99;                          // writes into freed memory -> UB
  std::cout << cloud[0].x << "\n";
}
