#include <ros/ros.h>

#include <gtest/gtest.h>

TEST(PlanValidityTest, InvalidGlobalPlanSizeWillFail) {
  //
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_interface");
  ros::NodeHandle nh;

  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}