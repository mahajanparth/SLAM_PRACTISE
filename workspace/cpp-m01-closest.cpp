// Module 1 — Exercise. Fill in the TODOs. Pick the right signatures: const, &, pointers.
// Run: ./run.sh scratch/cpp/m01_exercise.cpp
#include <cmath>
#include <limits>
#include <vector>

struct Point3 { double x, y, z; };

// (a) Euclidean range of a point from the sensor origin.
double range(const Point3& p) {
  // TODO

  return std::pow((std::pow((p.x),2.0) + std::pow((p.y),2.0) +std::pow((p.z),2)),0.5);
}

// (b) Scale every point in place by s (no copies of the cloud, no copies of points).
void scaleInPlace(std::vector<Point3>& cloud, double s) {
  // TODO
  for( auto & p : cloud){
    p.x*=s;
    p.y*=s;
    p.z*=s;
    
  }
}

// (c) Return a pointer to the point closest to q, or nullptr if the cloud is empty.
//     Why a pointer here and not a reference? Why `const Point3*`?
const Point3* findClosest(const std::vector<Point3>& cloud, const Point3& q) {
  // TODO
  double closest_val = std::numeric_limits<double>::max();
  const Point3* ptr = nullptr;
  for(auto&p : cloud){
    
    auto l2_norm=std::pow((std::pow((p.x-q.x),2.0) + std::pow((p.y-q.y),2.0) +std::pow((p.z-q.z),2)),0.5);
    
    if ( l2_norm <closest_val){
      ptr = &p;
      closest_val =l2_norm;
    }
  }
  return ptr;
}

// (d) Total length of the polyline cloud[0] -> cloud[1] -> ... -> cloud[n-1].
//     Must be safe for empty and single-point clouds (watch size_t underflow!).
double pathLength(const std::vector<Point3>& cloud) {
  
  if(cloud.size()<2){
    return 0.0;
  }


  double total_length = 0.0;

  for (std::size_t i = 1; i < cloud.size(); ++i) {

      double dx = cloud[i].x - cloud[i-1].x;
      double dy = cloud[i].y - cloud[i-1].y;
      double dz = cloud[i].z - cloud[i-1].z;

      total_length += std::sqrt(dx*dx + dy*dy + dz*dz);
  }

    return total_length;
}
