/**
 * @file tony_local_planner.cpp
 * @author Tony Guo (tonyguo@gyro.com.tw)
 * @author Jacky tseng (jacky.tseng@gyro.com.tw)
 *
 * @brief TonyLocalPlanner implementation
 * @copyright Copyright (c) 2020
 *
 */

#include <Eigen/Geometry>
#include <algorithm>
#include <boost/format.hpp>
#include <boost/iterator/zip_iterator.hpp>
#include <boost/math/constants/constants.hpp>
#include <iterator>
#include <regex>
#include <sstream>

#include <pluginlib/class_list_macros.h>
#include <tf/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tony_local_planner/tony_local_planner.h"

PLUGINLIB_EXPORT_CLASS(tony_local_planner::TonyLocalPlanner, nav_core::BaseLocalPlanner)

namespace {

constexpr auto PI = boost::math::constants::pi<double>();

/**
 * @brief This function calculates menger curvature of given three points
 *
 * @details The result of the global planner may contain curvatures. The local planner deals with the curvatures
 *          by finding the menger curvature of current desired point and the next two points, if the curvature of
 *          the three points is small, then we can assume that the path formed by these three points is approx.
 *          a line and can be followed by the MR without many issues.
 *
 * @param t_a   Point A
 * @param t_b   Point B
 * @param t_c   Point C
 * @return The menger curvature of point A, B, C
 *
 * @note  The sign of the area is determined according to the sequence of the point and the orientation or the
 *        coordinate system
 */
[[gnu::warn_unused_result]] inline double calculate_menger_curvature(geometry_msgs::PoseStamped const &t_a,
                                                                     geometry_msgs::PoseStamped const &t_b,
                                                                     geometry_msgs::PoseStamped const &t_c) noexcept {
  using boost::geometry::area;
  using boost::geometry::distance;

  auto const pts      = {t_a, t_b, t_c, t_a};  // ring or ploygon can be non-closed, thus specification is needed
  auto const triangle = tony_local_planner::Polygon_t(std::begin(pts), std::end(pts));

  return 4 * std::abs(area(triangle)) / (distance(t_b, t_a) * distance(t_a, t_c) * distance(t_c, t_b) + 1e-6);
}

/**
 * @brief This function calculates the angle of the slope given two poses
 *
 * @param t_prev  Previous pose
 * @param t_current Current pose
 * @return Angle in radius
 */
[[gnu::warn_unused_result]] double slope_angle(geometry_msgs::PoseStamped const &t_prev,
                                               geometry_msgs::PoseStamped const &t_current) noexcept {
  auto const dx = t_current.pose.position.x - t_prev.pose.position.x;
  auto const dy = t_current.pose.position.y - t_prev.pose.position.y;

  return std::atan2(dy, dx);
}

/**
 * @brief This function calculates the slope given two poses
 *
 * @param t_prev  Previous pose
 * @param t_current Current pose
 * @return The slope of the line from previous pose to current pose
 */
[[gnu::warn_unused_result]] constexpr double calculate_slope(geometry_msgs::PoseStamped const &t_prev,
                                                             geometry_msgs::PoseStamped const &t_current) noexcept {
  auto const dx = t_current.pose.position.x - t_prev.pose.position.x;
  auto const dy = t_current.pose.position.y - t_prev.pose.position.y;

  return dy / (dx + 1e-6);
}

/**
 * @brief This function constraints the input angle to [-\pi, \pi]
 *
 * @param angle Input angle in radian
 * @return Constrained angle
 */
[[gnu::warn_unused_result]] constexpr double constrain_angle(double const angle) noexcept {
  if (angle > PI) {
    return angle - 2 * PI;
  }

  if (angle < -PI) {
    return angle + 2 * PI;
  }

  return angle;
}

/**
 * @brief This function calculates the minimum distance to the line segment
 *
 * @param t_seg Line segment
 * @param t_current_pose  Pose to find the minimum distance to the line segment
 * @return Minimum distance from current pose to the line segment
 */
[[gnu::warn_unused_result]] double distance_to_line(tony_local_planner::Segment_t const &t_seg,
                                                    geometry_msgs::PoseStamped const &t_current_pose) noexcept {
  auto const slope           = ::calculate_slope(t_seg.first, t_seg.second);
  auto const intercept       = t_seg.first.pose.position.y - slope * t_seg.first.pose.position.x;
  auto const distance_vector = (slope * t_current_pose.pose.position.x + intercept - t_current_pose.pose.position.y) /
                               -std::sqrt(std::pow(slope, 2) + 1);
  return std::abs(distance_vector);
}

/**
 * @brief This function parses string into array3d
 *
 * @param t_str Input string
 * @return converted array3d
 */
[[gnu::warn_unused_result]] Eigen::Array3d string_to_array3d(std::string const &t_str) noexcept {
  std::istringstream iss(t_str);
  char bracket, comma;
  double gain_1, gain_2, gain_3;

  iss >> bracket >> gain_1 >> comma >> gain_2 >> comma >> gain_3;
  return Eigen::Array3d{gain_1, gain_2, gain_3};
}

}  // anonymous namespace

namespace tony_local_planner {

Eigen::Array3d const TonyLocalPlanner::INTEGRAL_WINDUP_LIMIT = Eigen::Array3d{0.006, 0.006, 0.006};

void TonyLocalPlanner::initialize(std::string name, Transform_t *tf, costmap_2d::Costmap2DROS * /*unused*/) {
  this->tf_ = tf;

  ros::NodeHandle private_nh("~/" + name);
  ros::NodeHandle public_nh;

  this->odom_sub_ = public_nh.subscribe<nav_msgs::Odometry>("odom", 1, [this](auto const &msg) {
    boost::mutex::scoped_lock const lock(this->odom_lock_);
    this->base_odom_.header = msg->header;
    this->base_odom_.pose   = msg->pose.pose;
  });

  this->lookahead_point_pub_ = private_nh.advertise<Pose_t>("lookahead_point", 1);
  this->nearest_point_pub_   = private_nh.advertise<Pose_t>("nearest_point", 1);
  this->error_pub_           = private_nh.advertise<std_msgs::String>("error", 1);

  this->dsrv_   = std::move(std::make_unique<DynamicReconfigureServer>(private_nh));
  auto const cb = boost::bind(&TonyLocalPlanner::reconfigure_callback, this, _1, _2);
  this->dsrv_->setCallback(cb);

  this->velocity_pid_.set_anti_integral_windup_limit(INTEGRAL_WINDUP_LIMIT);

  ROS_INFO("TonyLocalPlanner initialized");
}

bool TonyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  cmd_vel = geometry_msgs::Twist{};

  try {
    auto const car_pose_map  = this->transform_to_map(this->get_odom());
    this->nearest_segment_   = this->find_nearest_segment(this->global_plan_line_, car_pose_map);
    this->lookahead_segment_ = this->find_lookahead_segment(this->global_plan_line_, car_pose_map);

    this->nearest_point_       = this->nearest_segment_->first;
    this->lookahead_point_     = this->lookahead_segment_->second;
    this->line_error_distance_ = ::distance_to_line(*this->nearest_segment_, car_pose_map);

    this->lookahead_point_pub_.publish(this->lookahead_point_);
    this->nearest_point_pub_.publish(this->nearest_point_);

    geometry_msgs::Twist path_tracking_velocity{};
    switch (this->state_) {
      case State::RotatingToStart:
        path_tracking_velocity = this->rotate_to_start(this->lookahead_point_);
        break;
      case State::Moving:
        path_tracking_velocity = this->move(this->nearest_point_, this->lookahead_point_);
        break;
      case State::RotatingToGoal:
        path_tracking_velocity = this->rotate_to_goal(this->ending_point_);
        break;
      default:
        return true;
    }

    auto const line_angle = ::slope_angle(this->nearest_point_, this->lookahead_point_);

    auto const current_line_angle_wrt_mr_heading = line_angle - detail::get_yaw(car_pose_map);
    auto const virtual_vel     = Eigen::Vector2d{path_tracking_velocity.linear.x, path_tracking_velocity.linear.y};
    auto const transformed_vel = Eigen::Rotation2Dd{current_line_angle_wrt_mr_heading} * virtual_vel;

    cmd_vel.linear.x  = transformed_vel[0];
    cmd_vel.linear.y  = transformed_vel[1];
    cmd_vel.angular.z = path_tracking_velocity.angular.z;

    ROS_DEBUG_NAMED("cmd velocity", "current state: %s, cmd_vel (Vx, Vy, Wz) = (%.3f, %.3f, %.3f)",
                    this->current_state_name().c_str(), cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    return true;
  } catch (tf::TransformException const &e) {
    ROS_ERROR_STREAM("TonyLocalPlanner encounters error during moving stage, reason: " << e.what());
    return false;
  }
}

bool TonyLocalPlanner::isGoalReached() noexcept { return state_ == State::Finished; }

/**
 * @details For non-zero planner frequency case, we re-evaluate state only when "new" goal come. If new goal comes,
 *          local planner state will be reset to State::RotatingToStart or RotatingToGoal, base on the condition.
 */
bool TonyLocalPlanner::setPlan(const PoseArray_t &global_plan) {
  using boost::geometry::distance;

  auto const result = this->check_global_plan(global_plan);
  if (not result_is_good(result)) {
    auto const err_str = boost::get<std::string>(result);
    std_msgs::String err_msg;
    err_msg.data = err_str;

    this->error_pub_.publish(err_msg);
    ROS_ERROR_STREAM(err_str);
    return false;
  }

  // in order to comply with bs constraint
  this->global_plan_      = boost::get<PoseArray_t>(result);
  this->global_plan_line_ = this->set_segment_from_plan(this->global_plan_);

  if (detail::is_pose_eq(this->ending_point_.pose, global_plan.back().pose)) {
    return true;
  }

  this->ending_point_ = global_plan.back();
  try {
    auto const car_pose_map = this->transform_to_map(this->get_odom());

    if (distance(car_pose_map, this->ending_point_) <= this->xy_goal_tolerance_) {
      ROS_INFO("at goal, RotatingToGoal");
      this->state_ = State::RotatingToGoal;
      return true;
    }

    this->state_ = State::RotatingToStart;
    ROS_INFO("RotatingToStart");
    return true;
  } catch (tf::TransformException const &e) {
    ROS_ERROR_STREAM("TonyLocalPlanner::setPlan error, reason: " << e.what());
  }

  return false;
}

/**
 * @details This function do regex check to prevent user modifies the gains in yaml file directly but doing it wrong
 */
Eigen::Matrix3d TonyLocalPlanner::parse_gain_from_param_server() {
  ros::NodeHandle nh("~/");
  Eigen::Matrix3d ret_val;

  std::regex const pattern(R"(^\[(\s*\d+(\.\d+)?)(\s*\,\s*\d+(\.\d+)?\s*){2}\]$)");
  auto const x_pid   = nh.param<std::string>("TonyLocalPlanner/x_direction_pid", DEFAULT_X_PID_GAIN);
  auto const y_pid   = nh.param<std::string>("TonyLocalPlanner/y_direction_pid", DEFAULT_Y_PID_GAIN);
  auto const yaw_pid = nh.param<std::string>("TonyLocalPlanner/yaw_direction_pid", DEFAULT_THETA_PID_GAIN);

  ROS_DEBUG_STREAM_NAMED("gain", "x direction pid loaded: " << x_pid);
  ROS_DEBUG_STREAM_NAMED("gain", "y direction pid loaded: " << y_pid);
  ROS_DEBUG_STREAM_NAMED("gain", "yaw direction pid loaded: " << yaw_pid);

  std::cmatch c;
  if (not std::regex_match(x_pid.c_str(), c, pattern) or not std::regex_match(y_pid.c_str(), c, pattern) or
      not std::regex_match(yaw_pid.c_str(), c, pattern)) {
    throw std::invalid_argument("Wrong gain syntax, make sure that the patterns matched the desired form");
  }

  ret_val.row(0) = ::string_to_array3d(x_pid);
  ret_val.row(1) = ::string_to_array3d(y_pid);
  ret_val.row(2) = ::string_to_array3d(yaw_pid);
  return ret_val;
}

controller::PIDController<3> TonyLocalPlanner::init_controller() {
  Eigen::Matrix3d gain_matrix = parse_gain_from_param_server();
  controller::PIDController<3> ret_val{TonyLocalPlanner::SAMPLING_TIME, Eigen::Array3d{gain_matrix.col(0)},
                                       Eigen::Array3d{gain_matrix.col(1)}, Eigen::Array3d{gain_matrix.col(2)}};

  return ret_val;
}

/**
 * @details return a copy so that we can only read a copied variable. BTW, RVO is not a concern here, base_odom_ must be
 *          copied since we return by value instead of by reference.
 */
Pose_t TonyLocalPlanner::get_odom() noexcept {
  boost::mutex::scoped_lock const lock(this->odom_lock_);
  return this->base_odom_;
}

/**
 * @todo    This can be extended to modify the global plan if it doens't comply to the requirement
 */
Expected<PoseArray_t> TonyLocalPlanner::check_global_plan(PoseArray_t const &t_global) const noexcept {
  auto ret_val = PoseArray_t{t_global.begin(), std::prev(t_global.end())};

  if (t_global.size() < PREFERRED_MINIMUM_GLOBAL_PLAN_SIZE) {
    if (t_global.size() == REQUIRED_MIMIMUM_GLOBAL_PLAN_SIZE) {
      // @note can't generate useful test case for this if branch ATM
      ret_val.push_back(t_global.back());
      return ret_val;
    }

    return (boost::format("Invalid global plan size: %1%") % t_global.size()).str();
  }

  auto const zip_iter_end = boost::make_zip_iterator(boost::make_tuple(std::prev(ret_val.end()), ret_val.end()));
  auto zip_iter_begin     = boost::make_zip_iterator(boost::make_tuple(ret_val.begin(), std::next(ret_val.begin())));

  auto const duplicate_position = std::find_if(zip_iter_begin, zip_iter_end, [](auto const &t_point_pair) {
    geometry_msgs::PoseStamped first_pose, second_pose;
    boost::tie(first_pose, second_pose) = t_point_pair;

    geometry_msgs::Point first  = first_pose.pose.position;
    geometry_msgs::Point second = second_pose.pose.position;

    return detail::is_position_eq(first, second);
  });

  // ignored ATM, @todo need to generate test case to make sure this works
  if (duplicate_position != zip_iter_end) {
    // auto const idx = std::distance(t_global.begin(), boost::get<0>(duplicate_position.get_iterator_tuple()));
    // return (boost::format("Found duplicate waypoints in global plan at %1% and %2%") % idx % (idx + 1)).str();
  }

  return ret_val;
}

/**
 * @details TonyLocalPlanner decomposes every start to end movement into three stages: starting angle alignment
 *          (rotate_to_start), move to end point (move), and align to end angle (rotate_to_goal).
 */
geometry_msgs::Twist TonyLocalPlanner::rotate_to_start(Pose_t const &t_goal) {
  auto const goal_pose_odom    = this->transform_to_base_odom(t_goal);
  auto const yaw_diff          = ::constrain_angle(detail::get_yaw(goal_pose_odom) - detail::get_yaw(this->get_odom()));
  auto const in_tolerance_pred = [](auto const t_in_margin) { return t_in_margin; };

  this->record_error_status(std::abs(yaw_diff) <= this->yaw_goal_tolerance_);
  geometry_msgs::Twist ret_val{};
  if (std::all_of(this->error_window_.begin(), this->error_window_.end(), in_tolerance_pred)) {
    this->state_ = State::Moving;
    this->velocity_pid_.reset();
    this->error_window_.clear();

    ROS_INFO("Moving");
    return ret_val;
  }

  auto const error    = Eigen::Array3d{0.0, 0.0, yaw_diff};
  auto const velocity = this->velocity_pid_.calculate_control_output(error);
  ret_val.angular.z   = velocity[2];

  ROS_DEBUG_STREAM_NAMED("distance_error", "(x, y, w): " << error.transpose());
  return ret_val;
}

/**
 * @details TonyLocalPlanner decomposes every start to end movement into three stages: starting angle alignment
 *          (rotate_to_start), move to end point (move), and align to end angle (rotate_to_goal).
 *
 *          In order to track the line, we need to know the relative direction bewteen the line and MR, this is dealt
 *          with by taking the dot product and cross product of two vectors, (target position, start position) and
 *          (target position, current position), and we get the following facts:
 *
 *          - if dot product > 0: the goal is in front of the car
 *                           = 0: the goal is parallel to the car (moving direction is paralleled with line slope angle)
 *                           < 0: the goal is behind the car
 *          - if cross product > 0: the line is at the left hand side of the car (take moving direction as MR heading)
 *                             = 0: MR is right on the path
 *                             < 0: the line is at the right hand side of the car
 */
geometry_msgs::Twist TonyLocalPlanner::move(Pose_t const &t_nearest, Pose_t const &t_goal) {
  using boost::geometry::distance;

  auto const car_pose_odom     = this->get_odom();
  auto const goal_pose_odom    = this->transform_to_base_odom(t_goal);
  auto const nearest_pose_odom = this->transform_to_base_odom(t_nearest);

  auto const line    = Eigen::Vector3d{goal_pose_odom.pose.position.x - nearest_pose_odom.pose.position.x,
                                    goal_pose_odom.pose.position.y - nearest_pose_odom.pose.position.y, 0};
  auto const heading = Eigen::Vector3d{goal_pose_odom.pose.position.x - car_pose_odom.pose.position.x,
                                       goal_pose_odom.pose.position.y - car_pose_odom.pose.position.y, 0};
  auto const x_dir   = [line, heading]() {
    auto const dot_product   = line.dot(heading);
    auto const angle_heading = dot_product / (line.norm() * heading.norm());  // cos() = angle_heading
    if (std::abs(angle_heading) > 1e-2) {
      return dot_product > 0 ? 1.0 : -1.0;
    }

    return 0.0;
  }();
  auto const y_dir = heading.cross(line)[2] > 0 ? -1.0 : 1.0;

  auto const yaw_diff = ::constrain_angle(detail::get_yaw(goal_pose_odom) - detail::get_yaw(car_pose_odom));
  auto const distance_to_next_heading = distance(car_pose_odom, goal_pose_odom);
  ROS_DEBUG_NAMED("distance_error", "distance remain: %f", distance_to_next_heading);

  geometry_msgs::Twist ret_val{};
  this->record_error_status(distance_to_next_heading <= this->xy_goal_tolerance_);
  // the first if statement is not redundant if we want lookahead algorithm to work properly
  if (this->lookahead_segment_ == std::prev(this->global_plan_line_.end())) {
    if (std::all_of(this->error_window_.begin(), this->error_window_.end(), [](auto const t_bool) { return t_bool; })) {
      this->error_window_.clear();
      this->velocity_pid_.reset();

      this->state_ = State::RotatingToGoal;

      ROS_INFO("RotatingToGoal");
      return ret_val;
    }
  }

  // ROS_INFO_STREAM_COND(this->in_tuning_mode_, "dx: " << distance_to_next_heading
  //                                                       << "dy: " << this->line_error_distance_
  //                                                       << "dtheta: " << yaw_diff);
  auto const direction = Eigen::Array3d{x_dir, y_dir, 1.0};
  auto const error     = Eigen::Array3d{distance_to_next_heading, this->line_error_distance_, yaw_diff};
  auto const velocity  = this->velocity_pid_.calculate_control_output(direction * error);
  ROS_DEBUG_STREAM_NAMED("distance_error", "(x, y, w): " << error.transpose());

  ret_val.linear.x = velocity[0];
  ret_val.linear.y = velocity[1];

  ROS_DEBUG_NAMED("Velocity direction", "x_dir: %.1f, y_dir: %.1f", x_dir, y_dir);
  if (std::abs(yaw_diff) > this->move_yaw_goal_tolerance_) {
    ret_val.angular.z = velocity[2];
  }
  return ret_val;
}

/**
 * @details TonyLocalPlanner decomposes every start to end movement into three stages: starting angle alignment
 *          (rotate_to_start), move to end point (move), and align to end angle (rotate_to_goal).
 *
 *  @note   There are two possible situations to approach to this state, one is during setPlan, if the distance between
 *          goal point and the starting point is within the margin , then we only align with end pose, and thus skipped
 *          the two move states (rotate_to_start and move); while the other one is normal route (rotate_to_start -> move
 *          -> rotate_to_goal).
 */
geometry_msgs::Twist TonyLocalPlanner::rotate_to_goal(Pose_t const &t_goal) {
  auto const goal_pose_odom = this->transform_to_base_odom(t_goal);
  auto const yaw_diff       = ::constrain_angle(detail::get_yaw(goal_pose_odom) - detail::get_yaw(this->get_odom()));

  geometry_msgs::Twist ret_val{};
  this->record_error_status(std::abs(yaw_diff) <= this->yaw_goal_tolerance_);
  if (std::all_of(this->error_window_.begin(), this->error_window_.end(), [](auto const t_bool) { return t_bool; })) {
    this->velocity_pid_.reset();
    this->error_window_.clear();
    this->state_ = State::Finished;

    ROS_INFO("Goal reached");
    return ret_val;
  }

  auto const error    = Eigen::Array3d{0.0, 0.0, yaw_diff};
  auto const velocity = this->velocity_pid_.calculate_control_output(error);
  ret_val.angular.z   = velocity[2];

  ROS_DEBUG_STREAM_NAMED("distance_error", "(x, y, w): " << error.transpose());
  return ret_val;
}

/**
 * @details
 */
SegmentArray_t TonyLocalPlanner::set_segment_from_plan(PoseArray_t const &t_global) const noexcept {
  auto const zip_iter_end = boost::make_zip_iterator(boost::make_tuple(std::prev(t_global.end()), t_global.end()));
  auto zip_iter_begin     = boost::make_zip_iterator(boost::make_tuple(t_global.begin(), std::next(t_global.begin())));

  SegmentArray_t ret_val(static_cast<std::size_t>(std::distance(zip_iter_begin, zip_iter_end)));

  std::transform(zip_iter_begin, zip_iter_end, ret_val.begin(), [](auto const &t_point_pair) {
    Pose_t start, end;
    boost::tie(start, end) = t_point_pair;
    return Segment_t{start, end};
  });

  return ret_val;
}

/**
 * @details  Despite this function finds the global minimum, which may not be the one we are finding for. We are
 *           aware of off-track condition, thus it is guaranteed to find the correct nearest distance line segment.
 *
 *           If MR is not between starting point and end point of the global plan (e.g. moved away by joystick
 *           controller), is_off_track is meaningless and may be possibly true, result in the function return
 *           this->nearest_segment_.
 *
 *           During startup, where nearest_segment_ is not initialized yet, the situation mentioned above will left
 *           nearest_segment_ undefined (this->nearest_segment_ = this->nearest_segment_). To cope with it, this
 *           fucntion will return the starting segment if the MR doesn't reached starting point yet.
 *
 * @todo     Consider the other case when MR exceed end pose
 */
SegmentArrayConstPtr_t TonyLocalPlanner::find_nearest_segment(SegmentArray_t const &t_seg,
                                                              Pose_t const &t_curr) const noexcept {
  auto const dist_cmp = [&t_curr](auto const &t_lhs, auto const &t_rhs) {
    using boost::geometry::comparable_distance;

    auto const rhs_dist = comparable_distance(t_curr, t_rhs);
    auto const lhs_dist = comparable_distance(t_curr, t_lhs);
    return lhs_dist < rhs_dist;
  };

  auto const min_dist_segment = std::min_element(t_seg.begin(), t_seg.end(), dist_cmp);

  auto const dist_to_min_dist_seg = boost::geometry::comparable_distance(t_curr, *min_dist_segment);
  auto const dist_to_start_point  = boost::geometry::comparable_distance(t_curr, t_seg.front().first);
  if (dist_to_min_dist_seg == dist_to_start_point) {
    return min_dist_segment;
  }

  auto const is_off_track = boost::geometry::distance(t_curr, *min_dist_segment) > this->off_track_condition_;
  if (not is_off_track) {
    return min_dist_segment;
  }

  return this->nearest_segment_;
}

/**
 * @details The overall strategy for finding lookahead point is to put look ahead point as far as possible, until
 *          max lookahead distance is exceeded, then fallback until it doesn't exceed max curvature. Despite that we
 *          are dealing with straight line most of the time, we can almost always skip the first step, to make the
 *          algorithm extendable (e.g. tracking curves), we chose not to skip the first step
 */
SegmentArrayConstPtr_t TonyLocalPlanner::find_lookahead_segment(SegmentArray_t const &t_seg,
                                                                Pose_t const &t_curr) const noexcept {
  using boost::make_zip_iterator;

  auto const distance_geq_max = [max_lookahead = this->max_lookahead_distance_, &t_curr](auto const &t_in) {
    auto const distance_to_current_point = boost::geometry::distance(t_curr, t_in);
    return distance_to_current_point >= max_lookahead;
  };

  auto const max_dist_segment = std::find_if(this->nearest_segment_, std::prev(t_seg.cend()), distance_geq_max);
  if (max_dist_segment == this->nearest_segment_) {
    return max_dist_segment;
  }

  auto const zip_iter_end = make_zip_iterator(boost::make_tuple(std::prev(max_dist_segment), max_dist_segment));
  auto zip_iter_begin = make_zip_iterator(boost::make_tuple(this->nearest_segment_, std::next(this->nearest_segment_)));

  auto curvature_geq_max = [max_curvature = this->max_available_curvature_, curve_sum = 0.0](auto const &t_in) mutable {
    Segment_t side_1, side_2;
    boost::tie(side_1, side_2) = t_in;

    curve_sum += ::calculate_menger_curvature(side_1.first, side_1.second, side_2.second);
    return curve_sum >= max_curvature;
  };

  auto const max_curve_segment = std::find_if(zip_iter_begin, zip_iter_end, curvature_geq_max);

  return boost::get<1>(max_curve_segment.get_iterator_tuple());
}

/**
 * @details The timestamp used in waitForTransform is the timestamp from odometry pose, since the odometry is updated
 *          periodically through the callback, the timestamp is updated also.
 */
Pose_t TonyLocalPlanner::transform_to_base_odom(Pose_t const &t_source) {
  auto const odom_header   = this->get_odom().header;
  auto source_pose         = t_source;
  source_pose.header.stamp = odom_header.stamp;

  return detail::transform(this->tf_, source_pose, odom_header, this->transform_tolerance_);
}

/**
 * @details The timestamp used in waitForTransform is the timestamp from input pose, since the global plan is updated
 *          only before entering setPlan (see move_base source code for more deetails), i,e, the timestamp may get
 *          outdated pretty quick if the state doesn't change back to PLANNING.
 */
Pose_t TonyLocalPlanner::transform_to_map(Pose_t const &t_source) {
  auto map_header  = this->global_plan_.front().header;
  map_header.stamp = t_source.header.stamp;

  return detail::transform(this->tf_, t_source, map_header, this->transform_tolerance_);
}

void TonyLocalPlanner::record_error_status(bool const t_is_in_margin) noexcept {
  if (this->error_window_.size() >= this->steady_state_criterion_) {
    this->error_window_.pop_front();
  }
  this->error_window_.push_back(t_is_in_margin);
}

void TonyLocalPlanner::reconfigure_callback(TonyLocalPlannerConfig &config, uint32_t level) {
  ROS_INFO("TonyLocalPlanner reconfigure_callback, level: %d", level);

  this->steady_state_criterion_    = static_cast<std::size_t>(config.steady_state_criterion);
  this->xy_goal_tolerance_         = config.xy_goal_tolerance;
  this->yaw_goal_tolerance_        = config.yaw_goal_tolerance;
  this->move_yaw_goal_tolerance_   = config.move_yaw_goal_tolerance;
  this->max_available_curvature_   = config.max_available_curvature;
  this->max_lookahead_distance_    = config.max_lookahead_distance;
  this->off_track_condition_       = config.off_track_condition;
  this->off_track_error_condition_ = config.off_track_error_condition;

  Eigen::Array3d const bound{config.max_vel_x, config.max_vel_y, config.max_vel_move_theta};

  this->transform_tolerance_ = ros::Duration(config.transform_tolerance);
  this->velocity_pid_.set_control_limit(bound, -bound);
}

}  // namespace tony_local_planner
