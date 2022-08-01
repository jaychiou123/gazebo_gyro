/**
 * @file tf_wrapper.h
 * @author jacky.tseng@gyro.com.tw
 * @brief A small wrapper header for tony_local_planner so that it can be compiled with tf1 or tf2, according to the ros
 *        version
 *
 * @copyright Copyright (c) 2020
 */
#ifndef TF_WRAPPER_H_
#define TF_WRAPPER_H_

#include <Eigen/Geometry>
#include <boost/type_traits.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

namespace tf {

// forward declared to let compiler choose which type to use according to ROS_VERSION
class TransformListener;
class Quaternion;  // NOLINT, this is intended, for ros Kinetic forward declaration will catch this

}  // namespace tf

namespace tf2_ros {

// forward declared to let compiler choose which type to use according to ROS_VERSION
class Buffer;

}  // namespace tf2_ros

namespace tony_local_planner {

namespace detail {

template <typename T>
T rpy_to(double const t_row, double const t_pitch, double const t_yaw) noexcept {
  auto const rotation_mat = Eigen::AngleAxisd(t_row, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(t_pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(t_yaw, Eigen::Vector3d::UnitZ());

  return T{rotation_mat.x(), rotation_mat.y(), rotation_mat.z(), rotation_mat.w()};
}

template <typename T>
constexpr T geometry_msgs_quaternion_to(geometry_msgs::Quaternion const& t_quat) noexcept {
  return T{t_quat.x, t_quat.y, t_quat.z, t_quat.w};
}

template <typename T>
geometry_msgs::Quaternion to_geometry_msgs_quaternion(T const& t_quat) noexcept {
  geometry_msgs::Quaternion ret_val;
  ret_val.x = t_quat.x();
  ret_val.y = t_quat.y();
  ret_val.z = t_quat.z();
  ret_val.w = t_quat.w();
  return ret_val;
}

template <typename T>
double to_yaw(T const& t_quat) noexcept {
  auto const eigen_quat = Eigen::Quaterniond{t_quat.w(), t_quat.x(), t_quat.y(), t_quat.z()};

  auto const euler_angle = eigen_quat.toRotationMatrix().eulerAngles(0, 1, 2);
  return euler_angle[2];
}

static inline double get_yaw(geometry_msgs::PoseStamped const& t_quat) noexcept {
  auto const ros_quat   = t_quat.pose.orientation;
  auto const eigen_quat = Eigen::Quaterniond{ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z};

  auto const euler_angle = eigen_quat.toRotationMatrix().eulerAngles(0, 1, 2);
  return euler_angle[2];
}

template <typename T, typename... Args>
struct use_transform_pose {
  template <typename C, typename = decltype(std::declval<C>().transformPose(std::declval<Args>()...))>
  static std::true_type match_transform_pose_signature(int) noexcept;

  template <typename C>
  static std::false_type match_transform_pose_signature(...) noexcept;

  static constexpr bool value = decltype(match_transform_pose_signature<T>(0))::value;
};

template <typename T, template <typename...> class G, typename InOutType>
constexpr auto tf1_fn_caller = G<T, std::string, InOutType, InOutType&>::value;

template <typename T>
constexpr auto use_transform_pose_v = tf1_fn_caller<T, use_transform_pose, geometry_msgs::PoseStamped>;

/**
 * @brief
 *
 * @tparam T
 * @param t_tf
 * @param t_source
 * @param t_header
 * @return auto
 */
template <typename T, std::enable_if_t<use_transform_pose_v<T>, bool> = true>
auto transform(T* const t_tf, geometry_msgs::PoseStamped const& t_source, std_msgs::Header const& t_header,
               ros::Duration const& t_duration) {
  geometry_msgs::PoseStamped ret_val;
  t_tf->waitForTransform(t_header.frame_id, t_source.header.frame_id, t_header.stamp, t_duration);
  t_tf->transformPose(t_header.frame_id, t_source, ret_val);

  return ret_val;
}

/**
 * @brief
 *
 * @tparam T
 * @param t_tf
 * @param t_source
 * @param t_header
 * @return auto
 */
template <typename T, std::enable_if_t<!use_transform_pose_v<T>, bool> = true>
auto transform(T* const t_tf, geometry_msgs::PoseStamped const& t_source, std_msgs::Header const& t_header,
               ros::Duration const t_duration) {
  return t_tf->transform(t_source, t_header.frame_id, t_duration);
}

template <typename Alloc1, typename Alloc2>
bool is_position_eq(geometry_msgs::Point_<Alloc1> const& t_lhs, geometry_msgs::Point_<Alloc2> const& t_rhs,
                    double const t_eps = 1e-6) noexcept {
  return std::abs(t_lhs.x - t_rhs.x) <= t_eps and std::abs(t_lhs.y - t_rhs.y) <= t_eps and
         std::abs(t_lhs.z - t_rhs.z) <= t_eps;
}

template <typename Alloc1, typename Alloc2>
bool is_orientation_eq(geometry_msgs::Quaternion_<Alloc1> const& t_lhs, geometry_msgs::Quaternion_<Alloc2> const& t_rhs,
                       double const t_eps = 1e-6) noexcept {
  return std::abs(t_lhs.w - t_rhs.w) <= t_eps and std::abs(t_lhs.x - t_rhs.x) <= t_eps and
         std::abs(t_lhs.y - t_rhs.y) <= t_eps and std::abs(t_lhs.z - t_rhs.z) <= t_eps;
}

template <typename Alloc1, typename Alloc2>
bool is_pose_eq(geometry_msgs::Pose_<Alloc1> const& t_lhs, geometry_msgs::Pose_<Alloc2> const& t_rhs,
                double const t_eps = 1e-6) noexcept {
  return is_position_eq(t_lhs.position, t_rhs.position, t_eps) and
         is_orientation_eq(t_lhs.orientation, t_rhs.orientation, t_eps);
}

template <typename Alloc1, typename Alloc2>
bool is_posestamped_eq(geometry_msgs::PoseStamped_<Alloc1> const& t_lhs,
                       geometry_msgs::PoseStamped_<Alloc2> const& t_rhs, double const t_eps) noexcept {
  return is_pose_eq(t_lhs.pose, t_rhs.pose, t_eps) and t_lhs.header == t_rhs.header;
}

}  // namespace detail

}  // namespace tony_local_planner

#endif  // TF_WRAPPER_H_