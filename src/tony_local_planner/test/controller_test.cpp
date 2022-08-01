#include <gtest/gtest.h>

#include "utility/step_response_sim_helper.hpp"

// catkin run_tests --this | sed -En '/^-- run_tests.py/,/^-- run_tests.py/p'

TEST(controller_test, steady_state_error_MR) {
  constexpr auto P_GAIN          = 2.0;
  constexpr auto I_GAIN          = 1.0;
  constexpr auto SAMPLING_TIME   = 0.01;
  constexpr auto SIMULATION_TIME = 10.0;

  auto sim_helper   = StepResponseSimHelper{SAMPLING_TIME, SIMULATION_TIME};
  auto const model  = [=](auto const &t_state, auto const &t_input) { return t_input; };
  auto p_controller = controller::PIDController<>(0.01, P_GAIN);

  constexpr auto REFERENCE_CMD = 1.0;

  auto const sim_series_1 = sim_helper.simulate(model, p_controller, REFERENCE_CMD);

  EXPECT_NEAR(sim_helper.find_steady_state_error(sim_series_1), REFERENCE_CMD, 1e-3);
}

TEST(controller_test, rise_time_MR) {
  constexpr auto P_GAIN_1 = 2.0;
  constexpr auto P_GAIN_2 = 4.0;

  constexpr auto SAMPLING_TIME   = 0.01;
  constexpr auto SIMULATION_TIME = 10.0;

  auto const sim_helper = StepResponseSimHelper{SAMPLING_TIME, SIMULATION_TIME};
  auto const model      = [=](auto const &t_state, auto const &t_input) { return t_input; };

  constexpr auto REFERENCE_CMD = 1.0;

  auto p_controller_1     = controller::PIDController<>(SAMPLING_TIME, P_GAIN_1);
  auto const sim_series_1 = sim_helper.simulate(model, p_controller_1, REFERENCE_CMD);

  auto p_controller_2     = controller::PIDController<>(SAMPLING_TIME, P_GAIN_2);
  auto const sim_series_2 = sim_helper.simulate(model, p_controller_2, REFERENCE_CMD);

  EXPECT_GT(sim_helper.find_rise_time(sim_series_1, REFERENCE_CMD),
            sim_helper.find_rise_time(sim_series_2, REFERENCE_CMD));
}

TEST(controller_test, overshoot_MR) {
  //
}
