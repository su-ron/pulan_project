
#pragma once
#include "Eigen/Eigen"

namespace planning {

class ConvexSet {
 public:
  ConvexSet() {}
  ~ConvexSet() {}

 public:
  /**
   * @brief Given any two 2d point x1, x2, return its hyperplane.
   * @return The result stores a1, a2, b, where a1*x + a2*y = b, [x y]^T is the
   * vec on the hyperplane
   * @attention x1 and x2 cannot be 0 at the same time
   */
  static const Eigen::Vector3d Hyperplane2d(const Eigen::Vector2d& x1,
                                            const Eigen::Vector2d& x2) {
    double num = (x1 + x2).transpose() * x2;
    double den = x1.squaredNorm() + x2.squaredNorm();
    double t = num / den;
    Eigen::Vector2d a = t * x1 + (1 - t) * x2;
    double b = a.squaredNorm();
    Eigen::Vector3d hyperplane;
    hyperplane << a, b;
    return hyperplane;
  }
};

}  // namespace planning
