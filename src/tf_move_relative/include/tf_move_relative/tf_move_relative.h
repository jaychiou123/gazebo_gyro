#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <tf_move_relative/tf_MoveRelativeConfig.h>

class tf_MoveRelative
{
    public:
      tf_MoveRelative(std::string name);
      ~tf_MoveRelative();

      void reconfig_callback(tf_move_relative::tf_MoveRelativeConfig &config, uint32_t level);
      void execute(const move_base_msgs::MoveBaseGoalConstPtr &goal);

    private:
      ros::NodeHandle nh_;

      actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> action_server_;
      dynamic_reconfigure::Server<tf_move_relative::tf_MoveRelativeConfig> reconfigure_server_;

      tf2_ros::Buffer tf_Buffer_;
      tf2_ros::TransformListener *tf_listener_;

      ros::Publisher goal_pub_;
      ros::Publisher vel_pub_;

      struct velocity_setting
      {
          double max_vel;
          double min_vel;
          double acceleration;
      };

      velocity_setting x_, y_, theta_;

      std::string fixed_frame_;
      std::string base_frame_;

      double diff_initial_x;
      double diff_initial_y;
      double diff_initial_th;

      ros::Time time_initial;
      ros::Time now;
      double move_time_now;

      double rate_;
      double timeout_;
      double linear_tolerance_sq_, angular_tolerance_;

      double velocity;

      double cal_vel(double move_to_maxvel, double difference, velocity_setting &set);
      void stop_vel();
};