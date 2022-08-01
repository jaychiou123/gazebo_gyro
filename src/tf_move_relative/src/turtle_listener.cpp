#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "turtle_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;
  turtle.request.x = 0;
  turtle.request.y = 0;
  turtle.request.theta = 0;
  turtle.request.name = "turtle2";
  spawner.call(turtle);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);

  while (node.ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      ros::Time past = ros::Time::now() - ros::Duration(5.0);
      transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
      //transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", past, ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }
  return 0;
}