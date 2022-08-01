#include <tf_move_relative/tf_move_relative.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_move_relative");

  tf_MoveRelative move_relative(ros::this_node::getName());

  ros::spin();
  return 0;
}
