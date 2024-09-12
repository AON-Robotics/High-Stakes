#ifndef QUINTIC_PROFILE_HPP__
#define QUINTIC_PROFILE_HPP__

#include "./motion-profile.hpp"

namespace aon {

class QuinticMotionProfile : public MotionProfile {
 private:
  double v_0;
  double v_f;
  double coeffs[6] = {};

  double v(double t) {
    if (t < 0.0 || T < t) return 0;

    double output = 0;
    for (int i = 5; i > 0; i--) output = output * t + i * coeffs[i];

    return D * output;
  }

  double s(double t) {
    if (t < 0.0)
      return 0;
    else if (T < t)
      return D;

    double output = 0;
    for (int i = 5; i >= 0; i--) output = output * t + coeffs[i];

    return D * output;
  }

 public:
  double GetV0() { return v_0; }
  double GetVf() { return v_f; }

  void SetParams(double D_, double T_, double v_0_, double v_f_,
                 double correction_ratio_) {
    D = (D_ == 0.0) ? DBL_EPSILON : D_;
    T = (T_ == 0.0) ? DBL_EPSILON : std::abs(T_);
    v_0 = std::abs(v_0_) / D;
    v_f = std::abs(v_f_) / D;
    correction_ratio = std::max(0.0, std::min(correction_ratio_, 1.0));

    coeffs[0] = 0.0;
    coeffs[1] = v_0;
    coeffs[2] = 0;
    coeffs[3] = 10.0 / T - (4.0 * v_f + 6.0 * v_0);
    coeffs[3] /= T * T;
    coeffs[4] = (7.0 * v_f + 8.0 * v_0) - 15.0 / T;
    coeffs[4] /= T * T * T;
    coeffs[5] = 6.0 / T - 3.0 * (v_f + v_0);
    coeffs[5] /= T * T * T * T;
  }
};
};  // namespace aon

#endif  // QUINTIC_PROFILE_HPP__