#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import os
import yaml
from rospkg import RosPack
import rosparam
import rospy

from ddynamic_reconfigure_python.ddynamic_reconfigure import \
    DDynamicReconfigure

pid_result = {
    'x_direction_pid': '',
    'y_direction_pid': '',
    'yaw_direction_pid': ''
}


def syntax_check(gain):
    return bool(gain != '' and re.search(
        r'^\[(\s*\d+(\.\d+)?)(\s*\,\s*\d+(\.\d+)?\s*){2}\]$', gain))


def set_pid_result(modified, config):
    global pid_result

    value = config[modified]
    if syntax_check(value):
        rospy.logdebug('received option \'' + modified + '\': ' + value)
        pid_result[modified] = value
    else:
        rospy.logerr('Wrong syntax for option \'' + modified + '\': ' + value +
                     ', reset to its last valid value')
        config[modified] = pid_result[modified]


def gain_change_callback(config, level):
    set_pid_result('x_direction_pid', config)
    set_pid_result('y_direction_pid', config)
    set_pid_result('yaw_direction_pid', config)
    return config


if __name__ == "__main__":
    rospy.init_node('gain_tuning_helper', anonymous=True)
    rospy.loginfo(rospy.get_name())
    ddr = DDynamicReconfigure(rospy.get_name())

    local_planner_path = RosPack().get_path('tony_local_planner')
    robot_name = rospy.get_param('~robot')

    # robot_name not specified, maybe because we are launching it on remote
    # machine with suitable environment variable ROBOT
    if robot_name == 'NO_ENV_VAR__ROBOT__':
        robot_name = os.getenv('ROBOT')
    robot_dir = os.path.join(local_planner_path, 'robots', robot_name)

    controller_yaml_dir = os.path.join(robot_dir, 'controller.yaml')
    param_list, ns = rosparam.load_file(controller_yaml_dir)[0]
    pid_result = {key: param_list[key] for key in pid_result}

    ddr.add_variable(
        'x_direction_pid',
        'PID gain for tracking x direction distance, in the format: [p, i, d]',
        param_list['x_direction_pid'])
    ddr.add_variable(
        'y_direction_pid',
        'PID gain for tracking y direction distance, in the format: [p, i, d]',
        param_list['y_direction_pid'])
    ddr.add_variable(
        'yaw_direction_pid',
        'PID gain for tracking yaw difference, in the format: [p, i, d]',
        param_list['yaw_direction_pid'])
    ddr.start(gain_change_callback)

    rospy.spin()

    local_planner_param = rosparam.get_param('/move_base/TonyLocalPlanner')
    controller_param = {
        key: value
        for key, value in local_planner_param.items()
        if key != 'gain_tuning_helper'
    }

    tuning_result = local_planner_param['gain_tuning_helper']
    controller_param['x_direction_pid'] = tuning_result['x_direction_pid']
    controller_param['y_direction_pid'] = tuning_result['y_direction_pid']
    controller_param['yaw_direction_pid'] = tuning_result['yaw_direction_pid']

    # this didn't format the yaml file, which makes the file unreadable
    # rosparam.dump_params(controller_yaml_dir, '/move_base/TonyLocalPlanner')
    with open(controller_yaml_dir, 'w+') as f:
        f.seek(0)
        yaml.dump(controller_param, f, default_flow_style=False)
        f.truncate()
