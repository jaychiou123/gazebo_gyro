#ifndef DUMMY_GLOBAL_PLANNER_HPP__
#define DUMMY_GLOBAL_PLANNER_HPP__

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace dummy_global_planner {

class DummyGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  using Path = std::vector<geometry_msgs::PoseStamped>;

  DummyGlobalPlanner() = default;

  void initialize(std::string t_name, costmap_2d::Costmap2DROS *t_costmap_ros) {
    ros::NodeHandle public_nh;
    ros::NodeHandle private_nh_("~/" + t_name);

    this->plan_sub_ = public_nh.subscribe("recorded_path", 1, &DummyGlobalPlanner::path_sub_callback, this);
    this->plan_pub_ = private_nh_.advertise<nav_msgs::Path>("plan", 1);
  }

  bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan) {
    plan = current_path_.poses;

    nav_msgs::Path pub_val;
    pub_val.poses = plan;
    this->plan_pub_.publish(current_path_);

    return true;
  }

 private:
  void path_sub_callback(nav_msgs::PathConstPtr const &t_path) { this->current_path_ = *t_path; }

  nav_msgs::Path current_path_;

  ros::Subscriber plan_sub_;
  ros::Publisher plan_pub_;
};

}  // namespace dummy_global_planner

#endif  // DUMMY_GLOBAL_PLANNER_HPP__