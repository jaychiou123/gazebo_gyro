#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

std::string turtle_name;
void poseCallback(const turtlesim::PoseConstPtr& msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = turtle_name;
    transformStamped.transform.translation.x = msg->x;
    transformStamped.transform.translation.y = msg->y;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}


int main(int argc,char** argv){

    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle private_node("~");
    if (! private_node.hasParam("asdf"))
    {
         //ROS_INFO("turtle_name = %s", argv[0]);
        if (argc!= 2)
        {
            ROS_ERROR("need turtle name as argument");
            return -1;
        }
        turtle_name = argv[1];
        
       
    }
    else{
        private_node.getParam("asdf",turtle_name);
       ROS_INFO("turtle_name = %s", turtle_name.c_str());
       std::cout<<"turtle_name = "<<turtle_name.c_str();
    }
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(turtle_name+"/pose", 10 , &poseCallback);

    ros::spin();
    return 0;
}