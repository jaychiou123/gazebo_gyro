#!/usr/bin/env python
# -*- coding: utf-8 -*-

import shutil
import argparse
import os
import re
import yaml
from PIL import Image
from rospkg import RosPack


def directory(string):
    if not os.path.isdir(string):
        err_msg = 'no such directory: %s ' % string
        raise argparse.ArgumentTypeError(err_msg)
    return string


def match_pose_syntax(string):
    if not re.match(r'^\[(\s*\d+(\.\d+)?)(\s*\,\s*\d+(\.\d+)?\s*){2}\]$',
                    string):
        err_msg = '%s is not a valid syntax' % string
        raise argparse.ArgumentTypeError(err_msg)
    return re.findall(r'\s*(-?\d+(?:\.\d*)?)', string)


PKG_DIRECTORY = RosPack().get_path('tony_local_planner')
PKG_BOT_DIRECTORY = os.path.join(PKG_DIRECTORY, 'robots')

EXECUTABLE_DESCRIPTION = 'Setup simulation environment for your map'
MAP_DIR_DESCRIPTION = 'Directory to find scanned map picture'
MAP_NAME_DESCRIPTION = 'The name of the map'
ROBOT_DESCRIPTION = 'The name of the robot, defaulted to environmental variable ROBOT'
INIT_POSE_DESCRIPTION = 'Initial position of the robot, the value must be comma separated and enclosed with bracket, e.g. "[1,2,3]" (with quotation mark)'
DRY_RUN_DESCRIPTION = 'Print the result without executing the process'

parser = argparse.ArgumentParser(description=EXECUTABLE_DESCRIPTION)
parser.add_argument('--map-dir', help=MAP_DIR_DESCRIPTION, type=directory)
parser.add_argument('--map-name', help=MAP_NAME_DESCRIPTION, type=str)
parser.add_argument('--robot',
                    help=ROBOT_DESCRIPTION,
                    type=str,
                    default=os.getenv('ROBOT'))
parser.add_argument('--init-pos',
                    help=INIT_POSE_DESCRIPTION,
                    type=match_pose_syntax)
parser.add_argument('--dry-run',
                    action='store_true',
                    help=DRY_RUN_DESCRIPTION,
                    dest='dry_run')

args = parser.parse_args()

sim_map_dir = os.path.join(PKG_BOT_DIRECTORY, args.robot)
map_png_with_ext = args.map_name + '.png'
map_yaml_with_ext = args.map_name + '.yaml'

map_png_dir = os.path.join(args.map_dir, map_png_with_ext)
map_yaml_dir = os.path.join(args.map_dir, map_yaml_with_ext)


def generate_launch_file(init_pose):
    return """\
<launch>
  <node pkg="amcl" type="amcl" name="amcl" output="screen">
    <rosparam file="$(find tony_local_planner)/param/amcl.yaml" command="load" />

    <param name="initial_pose_x" value="%s"/>
    <param name="initial_pose_y" value="%s"/>
    <param name="initial_pose_a" value="%s"/>
  </node>

  <node name="map_server" pkg="map_server" type="map_server" args="$(arg sim_dir)/working_field.yaml" output="screen"/>
</launch>
""" % (init_pose[0], init_pose[1], init_pose[2])


def generate_world_file(map_name, img_size, resolution, origin, init_pose):
    plan_size = [size * resolution for size in img_size]
    pose_center = [
        origin[0] + plan_size[0] / 2.0, origin[1] + plan_size[1] / 2.0,
        origin[2]
    ]

    return """\
include "../../stage/robots/omnidir_robot.inc"
include "../../stage/environment.inc"

floorplan
(
  name "working_field"
  bitmap "working_field.png"
  size [ %f %f 2.000 ]
  pose [ %f %f %f 0.000 ]
)

omnidir_robot
(
  pose [ %s %s %s 0.00 ]
  name "robot"
)
""" % (plan_size[0], plan_size[1], pose_center[0], pose_center[1],
       pose_center[2], init_pose[0], init_pose[1], init_pose[2])


if __name__ == "__main__":
    if not os.path.isfile(map_png_dir):
        raise argparse.ArgumentTypeError('no such file')

    if not os.path.isfile(map_yaml_dir):
        raise argparse.ArgumentTypeError('no such file')

    map_info = {}
    img = Image.open(map_png_dir)
    with open(map_yaml_dir, 'r+') as f:
        map_info = yaml.load(f)
        origin = map_info['origin']
        resolution = map_info['resolution']

        world_file_dir = os.path.join(sim_map_dir, 'working_field_sim.world')
        world_file_content = generate_world_file(args.map_name, img.size,
                                                 resolution, origin,
                                                 args.init_pos)
        launch_file_dir = os.path.join(sim_map_dir, 'localization_sim.launch')
        launch_file_content = generate_launch_file(args.init_pos)

        print 'Generating world file for stage_ros: %s' % world_file_dir
        print 'Generating launch file: %s' % launch_file_dir

        if args.dry_run:
            print '\n\n****************************** START WORLD FILE *********************************\n'
            print world_file_content
            print '****************************** END WORLD FILE *********************************'

            print '\n\n****************************** START LAUNCH FILE *********************************\n'
            print launch_file_content
            print '****************************** END LAUNCH FILE *********************************'

        else:
            if not os.path.isdir(sim_map_dir):
                os.mkdir(sim_map_dir)
                print 'Create directory %s' % sim_map_dir
            else:
                print 'Directory %s exists, skip mkdir' % sim_map_dir

            with open(world_file_dir, 'w+') as world_f:
                world_f.write(world_file_content)

            with open(launch_file_dir, 'w+') as launch_f:
                launch_f.write(launch_file_content)

    test_set_dir = os.path.join(sim_map_dir, 'test_set')
    print 'Copy file %s from %s to %s' % (map_png_with_ext, args.map_dir,
                                          sim_map_dir)
    print 'Copy file %s from %s to %s' % (map_yaml_with_ext, args.map_dir,
                                          sim_map_dir)

    if not args.dry_run:
        dest_map_dir = os.path.join(sim_map_dir, 'working_field.png')
        dest_yaml_dir = os.path.join(sim_map_dir, 'working_field.yaml')

        shutil.copyfile(map_png_dir, dest_map_dir)

        with open(dest_yaml_dir, 'w+') as map_yaml:
            map_info['image'] = 'working_field.png'

            map_yaml.seek(0)
            yaml.dump(map_info, map_yaml)
            map_yaml.truncate()

        if not os.path.isdir(test_set_dir):
            os.mkdir(test_set_dir)
            print 'Create directory %s' % test_set_dir
        else:
            print 'Directory %s exists, skip mkdir' % test_set_dir
