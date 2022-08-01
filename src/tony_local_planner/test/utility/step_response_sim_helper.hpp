#ifndef STEP_RESPONSE_SIM_HELPER_HPP__
#define STEP_RESPONSE_SIM_HELPER_HPP__

#include <boost/iterator/zip_iterator.hpp>
#include <boost/tuple/tuple.hpp>
#include <cmath>
#include <iterator>
#include <numeric>
#include <vector>

#include "tony_local_planner/controller.h"

struct StepResponseSimHelper {
  double samplingTime;
  double simulationTime;

  template <typename T>
  auto simulate(T const &t_state_trans, controller::PIDController<> &t_controller, double const t_reference_cmd) const
    noexcept {
    int const sim_samples = this->simulationTime / this->samplingTime;
    std::vector<double> sim_result;
    sim_result.reserve(sim_samples);

    double state = 0.0;
    for (int i = 0; i < sim_samples; ++i) {
      sim_result.push_back(state);
      auto const error  = t_reference_cmd - state;
      auto const output = t_controller.calculate_control_output(error);

      state += t_state_trans(state, output) * this->samplingTime;
    }

    return sim_result;
  }

  auto find_settling_time(std::vector<double> const &t_sim_data, double const t_steady_state) const noexcept {
    auto const settling = std::find_if(t_sim_data.rbegin(), t_sim_data.rend(), [=](auto const &t_state) {
      return std::abs(t_steady_state - t_state) / t_steady_state > 0.05;
    });

    return std::distance(settling, std::prev(t_sim_data.rend())) * this->samplingTime;
  }

  auto find_steady_state_error(std::vector<double> const &t_sim_data) const noexcept {
    std::vector<double> d_sim_data(t_sim_data.size());
    std::adjacent_difference(t_sim_data.begin(), t_sim_data.end(), d_sim_data.begin(),
                             [](auto const &t_sec, auto const &t_first) { return std::abs(t_sec - t_first) / t_sec; });

    auto const end   = d_sim_data.end();
    auto const begin = d_sim_data.begin();

    auto const zip_iter_end = boost::make_zip_iterator(boost::make_tuple(end - 2, end - 1, end));
    auto zip_iter_begin     = boost::make_zip_iterator(boost::make_tuple(begin, begin + 1, begin + 2));

    auto const steady = std::find_if(zip_iter_begin, zip_iter_end, [](auto const &t_in) {
      double first, second, third;
      boost::tie(first, second, third) = t_in;
      return first < 0.001 and second < 0.001 and third < 0.001;
    });

    if (steady == zip_iter_end) {
      return t_sim_data.back();
    }

    return t_sim_data.back();
  }

  auto find_rise_time(std::vector<double> const &t_sim_data, double const t_steady_state) const noexcept {
    auto const low   = std::lower_bound(t_sim_data.begin(), t_sim_data.end(), 0.1 * t_steady_state);
    auto const upper = std::upper_bound(t_sim_data.begin(), t_sim_data.end(), 0.9 * t_steady_state);

    return std::distance(low, upper) * this->samplingTime;
  }

  auto find_percentage_overshoot(std::vector<double> const &t_sim_data, double const t_steady_state) const noexcept {
    auto const max = std::max_element(t_sim_data.begin(), t_sim_data.end());

    if (*max - t_steady_state > 1e-3) {
      return (*max - t_steady_state) / t_steady_state;
    }

    return 0.0;  // no overshoot (steady state error or overdamped)
  }
};

#endif