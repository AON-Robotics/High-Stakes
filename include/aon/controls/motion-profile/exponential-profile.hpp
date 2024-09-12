#ifndef EXPONENTIAL_PROFILE_HPP__
#define EXPONENTIAL_PROFILE_HPP__

#include "./motion-profile.hpp"

namespace aon {

class ExponentialMotionProfile : public MotionProfile {
 private:
  double v_0;

  double v(double t) {
    if (t < 0.0) return 0;
    return 5.0 * D / T * std::exp(-5.0 * t / T);
  }

  double s(double t) {
    if (t < 0.0) return 0;
    return D * (1 - std::exp(-5.0 * t / T));
  }

 public:
  double GetV0() { return v_0; }

  void SetParams(double D_, double T_, double correction_ratio_) {
    D = D_;
    T = (T_ == 0.0) ? DBL_EPSILON : std::abs(T_);
    v_0 = 5.0 * std::abs(D) / T;
    correction_ratio = std::max(0.0, std::min(correction_ratio_, 1.0));
  }
};
};  // namespace aon

#endif  // EXPONENTIAL_PROFILE_HPP__
