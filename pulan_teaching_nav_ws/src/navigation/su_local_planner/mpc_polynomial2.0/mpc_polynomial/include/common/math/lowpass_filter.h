#pragma once
#include <math.h>
#include <iostream>

namespace math {

/**
 * @brief   Second order butterworth filter
 * @param   dt Sampling time
 * @param   cutoff_frquency Cutoff frequency at -3db
 */
class Butterworth2ndFilter {
 public:
  Butterworth2ndFilter(const double& dt, const double& cutoff_frquency);
  Butterworth2ndFilter() = default;
  ~Butterworth2ndFilter() = default;

  void Initial(const double& dt, const double& cutoff_frequency);

  double filter(const double& u0);
  void Reset();

 private:
  double a0_;
  double a1_;
  double a2_;
  double b0_;
  double b1_;
  double b2_;

  double u1_;
  double u2_;
  double y1_;
  double y2_;
  bool first_hit_ = true;
  bool second_hit_ = true;
};

}  // namespace math
