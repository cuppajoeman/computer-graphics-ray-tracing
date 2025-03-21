#include "Triangle.h"
#include "Ray.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

// precond: det is non-zero
Eigen::Vector3d cramers_rule(const Eigen::Matrix3d& A, 
                             const Eigen::Vector3d& b) {

  assert(A.determinant() != 0);

  auto detA = A.determinant();

  Eigen::Vector3d x;

  Eigen::Matrix3d A1 = A;
  A1.col(0) = b;
  x(0) = A1.determinant() / detA;

  Eigen::Matrix3d A2 = A;
  A2.col(1) = b;
  x(1) = A2.determinant() / detA;

  Eigen::Matrix3d A3 = A;
  A3.col(2) = b;
  x(2) = A3.determinant() / detA;

  return x;
}

bool Triangle::intersect(
  const Ray & ray, const double min_t, double & t, Eigen::Matrix<double, 3, 1> & n) const
{
  Eigen::Vector3d pa = std::get<0>(corners);
  Eigen::Vector3d pb = std::get<1>(corners);
  Eigen::Vector3d pc = std::get<2>(corners);

  auto d1 = pa - pb;
  auto d2 = pa - pc;

  auto b = pa - ray.origin;


  Eigen::Matrix3d A;
  A << d1, d2, ray.direction;

  double detA = A.determinant();

  if (detA == 0) {
    return false;
  }

  auto solution = cramers_rule(A, b);

  auto beta = solution.x();
  auto gamma = solution.y();
  auto temp_t = solution.z();
  
  if (temp_t < min_t || gamma < 0 || gamma > 1 || beta < 0 || beta > (1-gamma)){
    return false;  
  } 

  t = temp_t;  
  n = d1.cross(d2).normalized();
  return true;
}
