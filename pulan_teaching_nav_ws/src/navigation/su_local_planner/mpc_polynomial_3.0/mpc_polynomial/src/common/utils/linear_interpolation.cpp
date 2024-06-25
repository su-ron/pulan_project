#include "common/utils/linear_interpolation.h"


namespace planning {

data_types::PncPoint LinearInterpolation::InterpolateByS(
    const Point &p0, const Point &p1, const double &s) {
  double ratio = (s - p0.s()) / (p1.s() - p0.s());

  double x = (1 - ratio) * p0.x() + ratio * p1.x();
  double y = (1 - ratio) * p0.y() + ratio * p1.y();
  double yaw =
      InterpolateForYaw(p0.yaw(), p0.s(), p1.yaw(), p1.s(), s);
  double kappa = (1 - ratio) * p0.kappa() + ratio * p1.kappa();
  double dkappa = (1 - ratio) * p0.dkappa() + ratio * p1.dkappa();
  double vel = (1 - ratio) * p0.motion_info().linear_velocity() +
               ratio * p1.motion_info().linear_velocity();
  double acc = (1 - ratio) * p0.motion_info().a() +
               ratio * p1.motion_info().a();
  double t = (1 - ratio) * p0.motion_info().t() +
             ratio * p1.motion_info().t();

  Point target_point = p0;
  target_point.set_x(x);
  target_point.set_y(y);
  target_point.set_yaw(yaw);
  target_point.set_s(s);
  target_point.set_kappa(kappa);
  target_point.set_dkappa(dkappa);
  target_point.mutable_motion_info()->set_linear_velocity(vel);
  target_point.mutable_motion_info()->set_a(acc);
  target_point.mutable_motion_info()->set_t(t);

  return target_point;
}

double LinearInterpolation::InterpolateForYaw(
    const double& a0, const double& t0, const double& a1,
    const double& t1, const double& t) {
  if (std::fabs(t1 - t0) <= 1e-5) {
    return math::NormalizeAngle(a0);
  }
  const double a0_n = math::NormalizeAngle(a0);
  const double a1_n = math::NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return math::NormalizeAngle(a);
}

}  // namespace planning

