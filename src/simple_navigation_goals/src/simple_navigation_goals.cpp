#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  double x_direction, z_qu, w_qu;
  ros::param::get("x_direction", x_direction);
  ros::param::get("z_qu", z_qu);
  ros::param::get("w_qu", w_qu);
  ROS_INFO("x_direction: %f", x_direction);
  //tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // move_base_msgs::MoveBaseGoal goal;

  // //we'll send a goal to the robot to move 1 meter forward
  // goal.target_pose.header.frame_id = "base_footprint";
  // goal.target_pose.header.stamp = ros::Time::now();

  // goal.target_pose.pose.position.x = x_direction;
  // goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(z_qu);
  // goal.target_pose.pose.orientation.z = z_qu;
  // goal.target_pose.pose.orientation.w = w_qu;
  // ROS_INFO("Sending goal");
  // ac.sendGoal(goal);

  // ac.waitForResult();

  // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //   ROS_INFO("Hooray, the base moved 1 meter forward");
  // else
  //   ROS_INFO("The base failed to move forward 1 meter for some reason");
  int i = 0;
  move_base_msgs::MoveBaseGoal goal;
  while(true){

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.stamp = ros::Time::now();
    if (i % 2 != 0){
      goal.target_pose.pose.position.x = 1.8;
      goal.target_pose.pose.orientation.w = 1;
    } 
    else{
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(z_qu);
    }
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
    i++;
}
  

  return 0;
}