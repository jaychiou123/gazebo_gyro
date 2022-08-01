#include<ros/ros.h>
#include <manipulator_class/manipulator_class.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "manipulate_control");
    Manipulator move_manipulate(ros::this_node::getName());
    ros::spin();

    return 0;
}