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
  // TODO
  return 0.0;
}

double sumField(const std::vector<Point3>& cloud, double Point3::*field) {
  // TODO
  return 0.0;
}

PointPredicate makeFilter(const std::string& name, double param) {
  // TODO: dispatch on name; return {} for unknown names
  return {};
}

std::vector<Point3> applyPipeline(const std::vector<Point3>& cloud,
                                  const std::vector<PointPredicate>& stages) {
  // TODO
  return {};
}
