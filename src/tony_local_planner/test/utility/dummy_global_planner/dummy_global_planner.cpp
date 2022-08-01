#include <condition_variable>
#include <mutex>
#include <pluginlib/class_list_macros.h>

#include "dummy_global_planner.h"

PLUGINLIB_EXPORT_CLASS(dummy_global_planner::DummyGlobalPlanner, nav_core::BaseGlobalPlanner)
