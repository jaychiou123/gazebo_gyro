#include <ros/ros.h>

#include "tony_local_planner/tony_local_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tony_local_planner");
  ros::NodeHandle nh{"move_base"};

  tony_local_planner::TonyLocalPlanner local_planner;
  local_planner.initialize("TonyLocalPlanner", nullptr, nullptr);

  ros::spin();

  return 0;
}