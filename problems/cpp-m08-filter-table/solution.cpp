#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <vector>

struct Point3 { double x, y, z; };

using UnaryOp = double (*)(double);
using PointPredicate = std::function<bool(const Point3&)>;

double applyN(UnaryOp f, double x, int n) {
  if (!f) return x;  // calling a null function pointer is UB, so guard it
  for (int i = 0; i < n; ++i) x = f(x);
  return x;
}

double sumField(const std::vector<Point3>& cloud, double Point3::*field) {
  double total = 0.0;
  for (const auto& p : cloud) total += p.*field;  // pointer-to-member selects x, y or z
  return total;
}

PointPredicate makeFilter(const std::string& name, double param) {
  // Dispatch table: name -> factory. Capture-less lambdas convert to plain function pointers.
  using Factory = PointPredicate (*)(double);
  static const std::map<std::string, Factory> table = {
      {"max_range", [](double r) -> PointPredicate {
         const double r2 = r * r;
         return [r2](const Point3& p) { return p.x * p.x + p.y * p.y + p.z * p.z <= r2; };
       }},
      {"min_z", [](double z) -> PointPredicate { return [z](const Point3& p) { return p.z >= z; }; }},
      {"max_z", [](double z) -> PointPredicate { return [z](const Point3& p) { return p.z <= z; }; }},
      {"min_x", [](double x) -> PointPredicate { return [x](const Point3& p) { return p.x >= x; }; }},
  };
  auto it = table.find(name);
  return it == table.end() ? PointPredicate{} : it->second(param);
}

std::vector<Point3> applyPipeline(const std::vector<Point3>& cloud,
                                  const std::vector<PointPredicate>& stages) {
  std::vector<Point3> out;
  out.reserve(cloud.size());
  for (const auto& p : cloud) {
    const bool keep = std::all_of(stages.begin(), stages.end(), [&p](const PointPredicate& s) {
      return !s || s(p);  // empty stage = skipped (calling it would throw bad_function_call)
    });
    if (keep) out.push_back(p);
  }
  return out;
}
