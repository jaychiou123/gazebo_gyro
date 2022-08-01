#include "tony_local_planner/tf_wrapper.h"
#include "utility/utility.hpp"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tony_local_planner/TonyLocalPlannerConfig.h>

namespace {

using TonyLocalPlannerCfg = dynamic_reconfigure::Client<tony_local_planner::TonyLocalPlannerConfig>;
using MoveBaseClient      = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

void action_feedback_cb(move_base_msgs::MoveBaseFeedbackConstPtr const& t_ptr,
                        std::shared_ptr<MoveBaseClient> t_client) {
  using namespace tony_local_planner::detail;
  static geometry_msgs::PoseStamped last_active_pose;
  if (is_pose_eq(last_active_pose.pose, t_ptr->base_position.pose)) {
    if (t_ptr->base_position.header.stamp.sec - last_active_pose.header.stamp.sec > 5.0) {
      t_client->cancelGoal();
    }
  } else {
    last_active_pose = t_ptr->base_position;
  }
}

auto load_bag_data() {
  auto const node_handle  = ros::NodeHandle{};
  auto const test_set_dir = node_handle.param("move_base/test_set_dir", std::string());
  auto const bag_player   = bag_util::BagPlayer<nav_msgs::Path>(bag_util::get_test_bag_files(test_set_dir));

  constexpr auto LOOKAHEAD_DIST_TO_TEST_PER_BAG = 2;
  std::vector<std::pair<std::vector<nav_msgs::Path>, double>> ret_val;
  ret_val.reserve(bag_player.size() * LOOKAHEAD_DIST_TO_TEST_PER_BAG);

  for (auto const& t_content : bag_player) {
    auto const interval = path_info::find_min_dist_between_pathpoint(t_content);
    auto const max_dist = path_info::find_max_dist_path(t_content);

    ret_val.push_back(std::make_pair(t_content.get(), interval));
    ret_val.push_back(std::make_pair(t_content.get(), max_dist));
  }

  return ret_val;
}

void set_lookahead(double const t_lookahead) {
  static TonyLocalPlannerCfg tony_local_planner_cfg{"/move_base/TonyLocalPlanner"};

  tony_local_planner::TonyLocalPlannerConfig current_cfg;
  tony_local_planner_cfg.getCurrentConfiguration(current_cfg);
  current_cfg.max_lookahead_distance = t_lookahead;
  tony_local_planner_cfg.setConfiguration(current_cfg);
}

auto run_plan(std::vector<nav_msgs::Path> const& t_bag_path) {
  static auto public_nh               = ros::NodeHandle{};
  static auto const plan_pub          = public_nh.advertise<nav_msgs::Path>("recorded_path", 1);
  static auto move_base_action_client = []() {
    constexpr auto WAIT_TIME = 5.0;
    MoveBaseClient* ret_val  = new MoveBaseClient("move_base", true);
    while (not ret_val->waitForServer(ros::Duration(WAIT_TIME))) {
    }

    return std::shared_ptr<MoveBaseClient>(ret_val);
  }();

  for (auto const path : t_bag_path) {
    plan_pub.publish(path);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = path.poses.back();
    move_base_action_client->sendGoal(goal, 0, 0, boost::bind(::action_feedback_cb, _1, move_base_action_client));
    move_base_action_client->waitForResult();

    if (move_base_action_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      break;
    }
  }

  return move_base_action_client->getState().toString();
}

TEST(GlobalPlanTrackTest, CanTrackPath) {
  auto const data = load_bag_data();

  for (auto const& plan : data) {
    set_lookahead(plan.second);
    ASSERT_EQ(run_plan(plan.first), "SUCCEEDED");
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_functionality");

  ros::AsyncSpinner spinner_{0};
  ros::NodeHandle nh;

  spinner_.start();
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}