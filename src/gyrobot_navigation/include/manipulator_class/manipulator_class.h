#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>



class Manipulator
{
    public:
        Manipulator(std::string name);
        ~Manipulator();
        void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);
    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
        move_base_msgs::MoveBaseFeedback feedback_;
        move_base_msgs::MoveBaseResult result_;
        ros::Publisher vel_pub_;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener *tfListener;
        ros::Time time_initial;
        ros::Time time_each;
        double diff_x;
        double diff_y;
        double diff_th;
        double velocity;
        struct velocity_setting
        {
            double max_vel  ;
            double min_vel ;
            double acceleration;
        };
        velocity_setting x_, y_, theta_;
        void stop_vel(); // make cmd_vel == 0
        double cal_vel(double difference, velocity_setting &set);
};