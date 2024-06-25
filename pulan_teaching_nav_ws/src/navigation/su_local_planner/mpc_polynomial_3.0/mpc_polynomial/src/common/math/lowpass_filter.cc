#include "common/math/lowpass_filter.h"

namespace math {

Butterworth2ndFilter::Butterworth2ndFilter(const double& dt,
                                           const double& cutoff_frquency) {
  Initial(dt, cutoff_frquency);
}

void Butterworth2ndFilter::Initial(const double& dt,
                                   const double& cutoff_frequency) {
  u1_ = 0;
  u2_ = 0;
  y1_ = 0;
  y2_ = 0;

  double wn = 2 * M_PI * cutoff_frequency;
  double m = 2 / dt;
  a0_ = m * m + wn * sqrt(2) * m + wn * wn;
  a1_ = 2 * wn * wn - 2 * m * m;
  a2_ = m * m - wn * sqrt(2) * m + wn * wn;
  b0_ = wn * wn;
  b1_ = 2 * wn * wn;
  b2_ = wn * wn;
}

double Butterworth2ndFilter::filter(const double& u0) {
  if (first_hit_ || second_hit_) {
    if (first_hit_) {
      first_hit_ = false;
      y2_ = 0;
      y1_ = u0;
      u2_ = 0;
      u1_ = u0;
    } else {
      second_hit_ = false;
      y2_ = y1_;
      y1_ = u0;
      u2_ = u1_;
      u1_ = u0;
    }
    return u0;
  }
  double y0 = (b0_ * u0 + b1_ * u1_ + b2_ * u2_ - a1_ * y1_ - a2_ * y2_) / a0_;
  y2_ = y1_;
  y1_ = y0;
  u2_ = u1_;
  u1_ = u0;

  return y0;
}
void Butterworth2ndFilter::Reset() {
  u1_ = 0;
  u2_ = 0;
  y1_ = 0;
  y2_ = 0;
  first_hit_ = true;
  second_hit_ = true;
}
}  // namespace math
