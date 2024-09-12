#pragma once

#include "./motion-profile.hpp"

namespace aon {

class HoldMotionProfile : public MotionProfile {
 private:
  double v(double t) {
    return -correction_ratio * (t + 2.0 * correction_ratio * T);
  }

  double s(double t) { return t / 3.0 + D - 2.0 * T / 3.0; }

 public:
  void SetParams(double correction_ratio_) {
    correction_ratio = correction_ratio_;
    T = 1E3;
    D = 144.0;
  }
};

};  // namespace aon