/**
 * @file controller.h
 * @author jacky.tseng@gyro.com.tw
 * @brief A small header only PID controller for basic PID control
 *
 * @copyright Copyright (c) 2020
 */
#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>
#include <cmath>
#include <type_traits>

namespace controller {

template <int VarNum = 1>
class PIDController {
 public:
  using Array  = Eigen::Array<double, VarNum, 1>;
  using Scalar = double;
  using OpType = std::conditional_t<VarNum == 1, Scalar, Array>;

 private:
  OpType m_gainP;
  OpType m_gainI;
  OpType m_gainD;

  OpType m_upperBound{0.0};
  OpType m_lowerBound{0.0};
  bool m_saturationEnabled = false;

  OpType m_errorAcumulated{0.0};
  double m_samplingTime;

  OpType m_integralLimit{0.0};
  bool m_antiIntegralWindupEnabled = false;

  OpType m_previousError{0.0};

  [[gnu::warn_unused_result]] static constexpr Scalar clamp(Scalar const t_input, Scalar const t_low,
                                                            Scalar const t_up) noexcept {
    return boost::algorithm::clamp(t_input, t_low, t_up);
  }

  [[gnu::warn_unused_result]] static constexpr Array clamp(Array const& t_input, Array const& t_low,
                                                           Array const& t_up) noexcept {
    return t_input.min(t_up).max(t_low);
  }

 public:
  explicit constexpr PIDController(double const t_sampling_time, OpType t_p, OpType t_i = OpType{0.0},
                                   OpType t_d = OpType{0.0}) noexcept
    : m_gainP(std::move(t_p)), m_gainI(std::move(t_i)), m_gainD(std::move(t_d)), m_samplingTime(t_sampling_time) {}

  constexpr void set_control_limit(OpType const& t_max, OpType const& t_min) noexcept {
    this->m_upperBound        = t_max;
    this->m_lowerBound        = t_min;
    this->m_saturationEnabled = true;
  }

  constexpr void disable_anti_integral_windup() noexcept { this->m_antiIntegralWindupEnabled = false; }

  constexpr void set_anti_integral_windup_limit(OpType const& t_value) noexcept {
    this->m_integralLimit             = t_value;
    this->m_antiIntegralWindupEnabled = true;
  }

  [[gnu::warn_unused_result]] constexpr OpType calculate_control_output(OpType const& t_error) noexcept {
    this->m_errorAcumulated += t_error * this->m_samplingTime;
    this->m_previousError = t_error;

    OpType const p_output = t_error * this->m_gainP;
    OpType i_output       = this->m_errorAcumulated * this->m_gainI;
    OpType const d_output = this->m_gainD * (t_error - this->m_previousError) / this->m_samplingTime;

    if (this->m_antiIntegralWindupEnabled) {
      i_output = clamp(i_output, -this->m_integralLimit, this->m_integralLimit);
    }

    OpType const result = p_output + i_output + d_output;
    if (this->m_saturationEnabled) {
      return clamp(result, this->m_lowerBound, this->m_upperBound);
    }

    return result;
  }

  constexpr void reset() noexcept {
    this->m_errorAcumulated = OpType{0.0};
    this->m_previousError   = OpType{0.0};
  }

  constexpr auto get_sampling_time() const noexcept { return this->m_samplingTime; }
};

}  // namespace controller

#endif  // CONTROLLER_H__