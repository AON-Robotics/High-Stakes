#ifndef TRAPEZOID_PROFILE_HPP__
#define TRAPEZOID_PROFILE_HPP__

#include "./motion-profile.hpp"

namespace aon {

class TrapezoidMotionProfile : public MotionProfile {
 private:
  double V;
  double v_0;
  double v_f;
  double a_0;
  double a_f;
  double t_0;
  double t_v;
  double t_f;
  double sign = 0;

  double speed_coeffs[3][2] = {};
  double position_coeffs[3][3] = {};

  bool ProcessParams() {
    V = std::abs(V);
    v_0 = std::abs(v_0);
    v_f = std::abs(v_f);
    a_0 = std::abs(a_0);
    a_f = std::abs(a_f);

    sign = (D > 0.0) ? 1.0 : (D < 0.0) ? -1.0 : 0.0;

    if (!std::isnormal(a_0)) return false;
    if (!std::isnormal(a_f)) return false;

    double numerator =
        a_f * v_0 * v_0 + a_0 * v_f * v_f + 2.0 * std::abs(D) * a_0 * a_f;
    double denominator = a_0 + a_f;
    const double max_speed = std::sqrt(numerator / denominator);

    // V = std::max(std::max(v_0, v_f), std::min(V, max_speed));
    V = std::min(V, std::abs(max_speed));

    if (!std::isnormal(V)) return false;

    numerator = std::abs(D) + (V - v_0) * (V - v_0) / (2.0 * a_0) +
                (V - v_f) * (V - v_f) / (2.0 * a_f);
    denominator = V;

    if (numerator < 0) return false;
    if (v_0 > V) return false;
    if (v_f > V) return false;

    T = numerator / denominator;
    t_0 = (V - v_0) / a_0;
    t_f = (V - v_f) / a_f;
    t_v = T - t_0 - t_f;

    if (t_v < 0) return false;

    // Stage 1: Constant acceleration
    speed_coeffs[0][0] = v_0;
    speed_coeffs[0][1] = a_0;
    position_coeffs[0][0] = 0.0;
    position_coeffs[0][1] = v_0;
    position_coeffs[0][2] = a_0 / 2.0;

    // Stage 2: Constant speed
    speed_coeffs[1][0] = V;
    speed_coeffs[1][1] = 0.0;
    position_coeffs[1][0] = -(V - v_0) * (V - v_0) / (2.0 * a_0);
    position_coeffs[1][1] = V;
    position_coeffs[1][2] = 0.0;

    // Stage 3: Constant decceleration
    speed_coeffs[2][0] = T * a_f + v_f;
    speed_coeffs[2][1] = -a_f;
    position_coeffs[2][0] = position_coeffs[1][0];
    position_coeffs[2][0] -= std::pow(T * a_f + v_f - V, 2.0) / (2.0 * a_f);
    position_coeffs[2][1] = T * a_f + v_f;
    position_coeffs[2][2] = -a_f / 2.0;

    return true;
  }

  double v(double t) {
    if (t < 0.0 || T < t) return 0;

    double output = 0.0;
    const int stage = (t <= t_0) ? 0 : (t <= t_0 + t_v) ? 1 : 2;

    output = speed_coeffs[stage][1] * t + speed_coeffs[stage][0];

    return sign * output;
  }

  double s(double t) {
    if (t < 0.0)
      return 0;
    else if (T < t)
      return D;

    double output = 0.0;
    const int stage = (t <= t_0) ? 0 : (t <= t_0 + t_v) ? 1 : 2;

    for (int i = 2; i >= 0; i--)
      output = output * t + position_coeffs[stage][i];

    return sign * output;
  }

 public:
  double GetV() { return V; }
  double GetV0() { return v_0; }
  double GetVf() { return v_f; }
  double GetA0() { return a_0; }
  double GetAf() { return a_f; }
  double GetT0() { return t_0; }
  double GetTv() { return t_v; }
  double GetTf() { return t_f; }

  void SetParams(double D_, double V_, double v_0_, double v_f_, double a_0_,
                 double a_f_, double correction_ratio_) {
    D = D_;
    V = V_;
    a_0 = a_0_;
    a_f = a_f_;
    v_0 = v_0_;
    v_f = v_f_;
    correction_ratio = std::max(0.0, std::min(correction_ratio_, 1.0));

    if (!ProcessParams()) {
      // If we can't use our custom values due to errors, force the entire
      // motion to just be constant velocity (stage 2);
      v_0 = v_f = V = V_;
      a_0 = a_f = 1.0;
      correction_ratio = 0.0;

      aon::logging::Error(
          "[X] We could not use our custom Motion Profile values");

      ProcessParams();
    }
  }
};

};  // namespace aon

#endif  // TRAPEZOID_PROFILE_HPP__
