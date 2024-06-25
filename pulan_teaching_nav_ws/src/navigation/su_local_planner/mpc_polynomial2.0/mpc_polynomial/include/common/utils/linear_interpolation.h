#pragma once
#include "common/data_types/pnc_point.h"


namespace planning {

class LinearInterpolation {
  typedef data_types::PncPoint Point;
 public:
  LinearInterpolation() = default;
  ~LinearInterpolation() = default;

  static Point InterpolateByS(const Point& p0, const Point& p1, const double& s);

  static double InterpolateForYaw(const double& a0, const double& t0,
                                  const double& a1, const double& t1,
                                  const double& t);
};

}  // namespace planning

