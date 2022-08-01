#!/bin/bash

[[ $1 == "NO_ENV_VAR__ROBOT__" ]] && robot_name=$ROBOT || robot_name=$1

# load the rest of the yaml file if that is needed
# rosparam load $(rospack find tony_local_planner)/param/move_base.yaml /move_base
rosparam load $(rospack find tony_local_planner)/robots/"$robot_name"/controller.yaml /move_base/TonyLocalPlanner

# we can run move_base node here so that the parameter loading is guaranteed to happen before the move_base initialization
# however, things will get a bit messy

# rosrun move_base move_base __name:=move_base
