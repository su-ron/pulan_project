#pragma once
#include <array>

namespace planning {

class QuinticPolynomial {
 public:
  QuinticPolynomial() = default;

  ~QuinticPolynomial() = default;

  QuinticPolynomial(const std::array<double, 3>& start,
                    const std::array<double, 3>& end,
                    const double param);

  QuinticPolynomial(const double x0, const double dx0, const double ddx0,
                    const double x1, const double dx1, const double ddx1,
                    const double param);

  QuinticPolynomial(const QuinticPolynomial& other);

  void SetParam(const double x0, const double dx0, const double ddx0,
                const double x1, const double dx1, const double ddx1,
                const double param);

  void IntegratedFromQuarticCurve(const QuinticPolynomial& other,
                                  const double init_value);

  double Evaluate (const std::uint32_t order, const double p) const;

  double ParamLength() const { return param_; }

  double Coef(const size_t order) const;

  size_t Order() const  { return 5; }

 private:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double x1, const double dx1, const double ddx1,
                           const double param);

  // f = sum(coef_[i] * x^i), i from 0 to 5
  std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_{{0.0, 0.0, 0.0}};
  std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};

  double param_ = 0.0;
};

}  // namespace planning

