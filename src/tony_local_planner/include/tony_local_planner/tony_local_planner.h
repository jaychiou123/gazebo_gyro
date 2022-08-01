/**
 * @file tony_local_planner.h
 * @author Tony Guo (tonyguo@gyro.com.tw)
 * @author Jacky tseng (jacky.tseng@gyro.com.tw)
 *
 * @brief A custom local planner for omnidirectional robot using pure pursuit algorithm
 *
 * @copyright Copyright (c) 2020
 */

#ifndef TONYLOCALPLANNER_H_
#define TONYLOCALPLANNER_H_

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/variant.hpp>

#include <atomic>
#include <deque>
#include <memory>
#include <string>

#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <ros/common.h>
#include <std_msgs/String.h>
#include <tony_local_planner/TonyLocalPlannerConfig.h>

#include "controller.h"
#include "tf_wrapper.h"

namespace boost {
namespace geometry {
namespace traits {

template <>
struct tag<geometry_msgs::PoseStamped> {
  using type = point_tag;
};

template <>
struct coordinate_type<geometry_msgs::PoseStamped> {
  using type = double;
};

template <>
struct coordinate_system<geometry_msgs::PoseStamped> {
  using type = boost::geometry::cs::cartesian;
};

template <>
struct dimension<geometry_msgs::PoseStamped> : public boost::mpl::int_<2> {};

template <std::size_t Dimension>
struct access<geometry_msgs::PoseStamped, Dimension> {
  static inline double get(geometry_msgs::PoseStamped const &t_pose) {
    if (Dimension == 0) {
      return t_pose.pose.position.x;
    }

    return t_pose.pose.position.y;
  }

  static inline void set(geometry_msgs::PoseStamped &t_pose, double const &value) {
    if (Dimension == 0) {
      t_pose.pose.position.x = value;
    } else {
      t_pose.pose.position.y = value;
    }
  }
};

}  // namespace traits
}  // namespace geometry
}  // namespace boost

namespace tony_local_planner {

using Pose_t      = geometry_msgs::PoseStamped;
using PoseArray_t = std::vector<Pose_t>;
using Segment_t   = boost::geometry::model::segment<Pose_t>;
using Polygon_t   = boost::geometry::model::ring<Pose_t>;

using SegmentArray_t         = std::vector<Segment_t>;
using SegmentArrayConstPtr_t = SegmentArray_t::const_iterator;

template <typename T>
using Expected = boost::variant<T, std::string>;

/**
 * @brief
 */
class TonyLocalPlanner final : public nav_core::BaseLocalPlanner {
 public:
  static constexpr auto REQUIRED_MIMIMUM_GLOBAL_PLAN_SIZE  = 2; /*!< global plan size less than this is invalid */
  static constexpr auto PREFERRED_MINIMUM_GLOBAL_PLAN_SIZE = 3; /*!< global plan size greater than this is preferable */
  static constexpr auto IS_MELODIC = ROS_VERSION >= ROS_VERSION_COMBINED(1, 14, 9); /*!< check nav_core impl version*/
  static constexpr auto SAMPLING_TIME = 0.05; /*!< sampling time for local planner (20 Hz) */

  static constexpr auto DEFAULT_X_PID_GAIN     = "[0.6, 0.7, 0.0]";   /*!< Default gain for x direction path tracking */
  static constexpr auto DEFAULT_Y_PID_GAIN     = "[0.7, 0.375, 0.0]"; /*!< Default gain for y direction path tracking */
  static constexpr auto DEFAULT_THETA_PID_GAIN = "[0.6, 0.5, 0.0]"; /*!< Default gain for yaw direction path tracking */

  static const Eigen::Array3d INTEGRAL_WINDUP_LIMIT; /*!< Anti integral windup limit */

  using Transform_t = std::conditional_t<IS_MELODIC, tf2_ros::Buffer, tf::TransformListener>;

  void initialize(std::string name, Transform_t *tf, costmap_2d::Costmap2DROS *costmap_ros) override;
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;
  bool isGoalReached() noexcept override;
  bool setPlan(const PoseArray_t &global_plan) override;

 private:
  /**
   * @brief This function checks if the result is expected or error
   *
   * @tparam T      Expected type, auto deduced by function argument
   * @param t_res   result
   *
   * @return true   The result is good
   * @return false  The result is error
   */
  template <typename T>
  static bool result_is_good(Expected<T> const &t_res) noexcept {
    return t_res.which() == 0;
  }

  using DynamicReconfigureServer = dynamic_reconfigure::Server<TonyLocalPlannerConfig>;

  /**
   * @brief This function parses gains from parameter server which are stored in string, this function will throw
   *        invalid argument if regex check failed, indicating the gain has the wrong format.
   *
   * @return gain represent in the following form
   *         _                 _
   *        |   x_p, x_i, x_d   |
   *        |                   |
   *        |   y_p, y_i, y_d   |
   *        |                   |
   *        |_  a_p, a_i, a_d  _|
   */
  static Eigen::Matrix3d parse_gain_from_param_server();

  static controller::PIDController<3> init_controller();

  /**
   * @brief This function obtain the tuning_mode from parameter server and delete it immediately
   *
   * @return true
   * @return false
   *
   * @note  By deleting the parameter immediately, we prevent anyone from altering the behaviour of the local planner
   */
  static bool is_in_tuning_mode() noexcept {
    auto private_nh    = ros::NodeHandle("~/");
    auto const ret_val = private_nh.param("tuning_mode", false);
    private_nh.deleteParam("tuning_mode");
    return ret_val;
  }

  /**
   * @brief This function returns current moving state name
   *
   * @return state name
   */
  std::string current_state_name() const noexcept {
    using namespace std::string_literals;
    switch (this->state_) {
      case State::RotatingToStart:
        return "RotatingToStart"s;
      case State::RotatingToGoal:
        return "RotatingToGoal"s;
      case State::Moving:
        return "Moving"s;
      case State::Finished:
        return "Finished"s;
    }

    return "UNKNOWN_STATE"s;
  }

  /**
   * @brief This function is the thread-safe getter function of base odom
   *
   * @return a copy of base odom
   */
  Pose_t get_odom() noexcept;

  /**
   * @brief This function checks if global plan satisfies the requirement, it checks for the size of the global plan and
   *        whether their are consecutive duplicate points
   *
   * @todo   Modify the global plan in order to satsify our need
   * @return PoseArray_t or error message if not satisfied
   */
  Expected<PoseArray_t> check_global_plan(PoseArray_t const &t_global) const noexcept;

  /**
   * @brief This function is one of the state functions during moving stage, it calculates only angular velocity from
   *        goal pose so that the pose of the MR aligns with the goal pose. This function will throw
   *        tf::TransformException if any transform failed
   *
   * @param t_goal  Pose to turn to
   * @return velocity at the moment, the velocity here is calculated from path-tracking point of view, which needs to
   *         be transformed to car command velocity
   */
  geometry_msgs::Twist rotate_to_start(Pose_t const &t_goal);

  /**
   * @brief This function is one of the state functions during moving stage, it calculates linear and angular
   *        velocity from goal pose in order to move the MR to the goal. This function will throw tf::TransformException
   *        if any transform failed
   *
   * @param t_nearest   Nearest point from current position
   * @param t_goal      Pose to move to
   * @return velocity at the moment, the velocity here is calculated from path-tracking point of view, which needs to
   *         be transformed to car command velocity
   */
  geometry_msgs::Twist move(Pose_t const &t_nearest, Pose_t const &t_goal);

  /**
   * @brief This function is one of the state functions during moving stage, it calculates only angular velocity from
   *        goal pose so that the pose of the MR aligns with the goal pose. This function will throw
   *        tf::TransformException if any transform failed
   *
   * @param t_goal    Pose to turn to
   * @return velocity at the moment, the velocity here is calculated from path-tracking point of view, which needs to
   *         be transformed to car command velocity
   */
  geometry_msgs::Twist rotate_to_goal(Pose_t const &t_goal);

  /**
   * @brief This function sets the line segment from global plan
   *
   * @param t_global    Global plan
   * @return Line segment array
   */
  SegmentArray_t set_segment_from_plan(PoseArray_t const &t_global) const noexcept;

  /**
   * @brief This function finds the nearest line segment to global plan from current pose
   *
   * @param t_seg     Global plan
   * @param t_curr    Current pose
   * @return Iterator pointing to the nearest line segment in the global plan
   */
  SegmentArrayConstPtr_t find_nearest_segment(SegmentArray_t const &t_seg, Pose_t const &t_curr) const noexcept;

  /**
   * @brief This function finds the nearest point to global plan from current pose
   *
   * @param t_seg     Global plan
   * @param t_curr    Current pose
   * @return Iterator pointing to the pose that satisfies lookahead criterion in the global plan
   */
  SegmentArrayConstPtr_t find_lookahead_segment(SegmentArray_t const &t_seg, Pose_t const &t_curr) const noexcept;

  /**
   * @brief This function transforms input pose to odometry frame, this function will throw everything
   *        TransformListener::waitForTransform and TransformListener::transformPose throw.
   *
   * @param t_source  input pose
   * @return input pose under odometry frame
   */
  Pose_t transform_to_base_odom(Pose_t const &t_source);

  /**
   * @brief This function transforms input pose to map frame, this function will throw everything
   *        TransformListener::waitForTransform and TransformListener::transformPose throw.
   *
   * @param t_source  input pose
   * @return input pose under map frame
   */
  Pose_t transform_to_map(Pose_t const &t_source);

  /**
   * @brief This function records the residual error every sampling interval
   *
   * @param t_is_in_margin  Error is in margin or not
   */
  void record_error_status(bool const t_is_in_margin) noexcept;

  void reconfigure_callback(TonyLocalPlannerConfig &config, uint32_t level);

  bool in_tuning_mode_ = is_in_tuning_mode();

  std::unique_ptr<DynamicReconfigureServer> dsrv_{nullptr};

  enum class State { RotatingToStart, Moving, RotatingToGoal, Finished };

  Transform_t *tf_ = nullptr;

  PoseArray_t global_plan_;
  SegmentArray_t global_plan_line_;

  Pose_t base_odom_;
  boost::mutex odom_lock_;

  ros::Subscriber odom_sub_;

  ros::Publisher lookahead_point_pub_;
  ros::Publisher nearest_point_pub_;
  ros::Publisher error_pub_;

  State state_ = State::Finished;

  SegmentArrayConstPtr_t nearest_segment_;
  SegmentArrayConstPtr_t lookahead_segment_;
  Pose_t nearest_point_;
  Pose_t lookahead_point_;
  Pose_t ending_point_;

  controller::PIDController<3> velocity_pid_ = init_controller();

  std::deque<bool> error_window_;

  double max_lookahead_distance_  = 0.0;
  double max_available_curvature_ = 0.0;

  double xy_goal_tolerance_       = 0.0;
  double yaw_goal_tolerance_      = 0.0;
  double move_yaw_goal_tolerance_ = 0.0;

  double off_track_condition_       = 0.0;
  double off_track_error_condition_ = 0.0;
  double line_error_distance_       = 0.0;

  std::size_t steady_state_criterion_ = 20;
  ros::Duration transform_tolerance_{0.0};
};

}  // namespace tony_local_planner

#endif /* TONYLOCALPLANNER_H_ */
