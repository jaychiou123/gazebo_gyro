# Tony Local Planner <!-- omit in toc -->

_A custom local planner for omnidirectional robot_

## Table of Contents

- [Table of Contents](#table-of-contents)
- [About The Project](#about-the-project)
  - [Built With](#built-with)
- [Getting Started](#getting-started)
  - [Prerequisite](#prerequisite)
  - [Installation](#installation)
- [Simulation](#simulation)
- [Unit Testing](#unit-testing)
  - [Incorporate global planner](#incorporate-global-planner)
- [Gain Tuning](#gain-tuning)
  - [Remote Setup](#remote-setup)
- [Public Interface](#public-interface)
  - [Published Topics](#published-topics)
  - [Subscribed Topic](#subscribed-topic)
  - [Executables](#executables)
  - [Parameters](#parameters)
- [Debugging](#debugging)
- [TODOs](#todos)
- [Disclaimer](#disclaimer)
- [TL, DR](#tl-dr)
- [Contact](#contact)
  - [Reference](#reference)

## About The Project

Choosing and adjusting local planner of others to fit our needs may not be easier than writing one from scratch. This project was born with this assumption in mind. The local planner performs like pure pursuit, but with more flexibility.

### Built With

-   [ROS Kinetic Kame](http://wiki.ros.org/kinetic) under [Ubuntu 16.04.6 LTS (Xenial Xerus)](http://releases.ubuntu.com/16.04/)
-   [ROS Melodic Morenia](http://wiki.ros.org/melodic) under [Ubuntu 18.04.5 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04.5/)

## Getting Started

### Prerequisite

In order to get the local planner up and running on the machine, the following third party libraries must be installed:

-   Eigen: 3.3.1 at least

    -   For Ubuntu 18.04, this can be easily done by `sudo apt install libeigen3-dev`
    -   For Ubuntu 16.04, eigen3 that satisfies the requirement needs to be installed from source. Download the latest version (or whatever version greater than required) [here](https://gitlab.com/libeigen/eigen/-/releases). After downloading and extracting to directory you like:

            cd <your eigen directory>
            mkdir build && cd build
            cmake ..
            make install

    This will install `eigen` to `/usr/local/include`, no additional steps needed since eigen is a header-only library.

-   Boost: 1.58 at least, this is the default version shipped with ROS Kinetic

### Installation

1.  Clone the repo to your catkin workspace

```sh
git clone https://git-codecommit.us-east-2.amazonaws.com/v1/repos/tony_local_planner
```

2.  Install dependencies

```sh
rosdep install tony_local_planner
```

3.  Build tony local planner

```sh
catkin clean tony_local_planner # this is required if tony_local_planner is built before
catkin build tony_local_planner
```

## Simulation

[stage_ros](http://wiki.ros.org/stage_ros) is used to simulate the behaviour of the local planner, to have the local planner up and running alongside with stage_ros and (optionally) rviz, run the following command:

```sh
roslaunch tony_local_planner simulation.launch robot:=<your robot>
```

Some options regarding simulation configuration:

-   robot: the directory name under `robots`, e.g. `ase-AMR-A-04L4`, `spil-AMR-A-04L4` etc., this is required argument, defaulted to the result of environment variable $(ROBOT)
-   global_planner: change this to see how local planner interacts with different global planner, defaulted to `global_planner/GlobalPlanner`
-   visualization: enable this to launch `rviz`, defaulted to `true`

**Note: Make sure there is no existing `amcl` or `map_server` node, otherwise the simulation will fail**

## Unit Testing

To build and run all unit tests, including `controller_test` and `local_planner_test` for each test set

```sh
catkin build tony_local_planner # this will build all test executables
catkin run_tests tony_local_planner --no-deps # this will run all registered tests, --no-deps here is to skip building test from other dependencies
```

`tony_local_planner` will find all test that can be run (by finding robot name under `robots` directory), tests will be executed parallelly. To run test for certain robot only:

```
rostest tony_local_planner functionality.test robot:=<your robot>
```

In addition to arg `robot`, there are some optional args for `functionality.test`:

-   `global_planner`: [see next section](#Incorporate-global-planner)
-   `visualization`: defaulted to `false`. Change to `true` to enable `rviz` visualization.

### Incorporate global planner

The unit test above uses dummy global planner, which only recieves plans from the bag file, to take global planner into consideration, simply change the option `global_planner` to the one you want to test with:

```sh
rostest tony_local_planner local_planner_test.test robot:=<your robot> global_planner:=<your global planner>
```

## Gain Tuning

PID control is used for path tracking, to make sure the gain is suitable for the MR under specific map, these gains can be tuned via dynamic reconfigure by running gain tuning mode:

```sh
roslaunch tony_local_planner gain_tuning.launch
```

This will enter gain tuning mode in local machine, no need to specify `robot`, with that said, this command is expected to run in the MR instead of simulation environment. However, if you do it by `ssh -X` or other remote connection approaches, you may suffer from poor internet connection or too much data jamming the internet, making the GUI not responsive at all. To mitigate gain tuning process from poor internet, the package provides remote gain tuning. Before doing so, there are some prep works need to be done, see [remote gain tuning setup](#Remote-Setup).

Upon launching, three windows will show up, i.e., rqt_reconfigure, rqt_plot, and rviz. In rqt_reconfigure, three entries, `x_direction_pid`, `y_direction_pid`, `yaw_direction_pid` can now be modifed. The format must be in the form of `[p_gain, i_gain, d_gain]`, invalid input will not be applied, with error log notifying the user. Also, any attempt to modify gains during non-tuning mode is invalid, the change will not be applied and will return to its default value immediately, with log warning invalid operation.

### Remote Setup

In order to utilize remote gain tuning, make sure that both your PC and the MR have the same version of tony_local_planner. Also, we need to gain access to remote machine without entering password, [see how to add ssh key fingerprint to known_hosts here](https://www.techrepublic.com/article/how-to-easily-add-an-ssh-fingerprint-to-your-knownhosts-file-in-linux/) (only this method will work, due to the mechanism of the remote roslaunch).

The gain tuning process requires local pc to launch the node to the master at remote machine, to do so (note: all the process should be done before roscore, for remote machine, put the following `export`s inside `$HOME/.bashrc`, or `$HOME/environment.sh` for gyro AMR product):

```sh
export ROS_MASTER_URI=http://<fixed ip or hostname>:<port>  # e.g. ROS_MASTER_URI=http://192.186.0.20:11311
                                                            # Note that this needs to be set on BOTH local PC AND remote machine
                                                            # DO NOT SET IT TO localhost IN THIS CASE, i.e., http://localhost:11311
```

Also, in order to make remote machine and local pc communicate with each other, this needs to be set on BOTH side:

```sh
export ROS_IP=<your ip> # e.g. On your machine export ROS_IP=192.168.0.132 and on your remote machine export ROS_IP=192.168.0.20
```

(Notice that ROS_IP and ROS_HOSTNAME are mutually exclusive, see [ROS environmental variables](http://wiki.ros.org/ROS/EnvironmentVariables))

Last but not least, the setup script (i.e. env-loader) in the remote machine. It must do at least the following things: 1. source the bash file just as you normally do to gain access to ros commands 2. execute the command that is passed via ssh, see [the tutorial](http://wiki.ros.org/roslaunch/XML/machine). A minimal example of the setup script will look like this:

```sh
# Create remote_setup.sh under /home/gyro with following content:
. ~/environment.sh

exec "$@"
```

After the setup, run:

```sh
roslaunch tony_local_planner gain_tuning.launch machine_ip:=<remote machine ip> setup_script:=<dir/to/remote/machine/setup/script> global_planner:=<desire global planner>
```

Then the GUIs will be opened on your side instead of the remote machine, while `move_base`, and `gain_tuning_helper` will be launched at remote machine.

Since these need to be done everytime you want to perform gain tuning and are quite complicated, tony_local_planner provides a simpler way of doing so:

```sh
rosrun tony_local_planner start_gain_tuning.sh --machine-ip=<remote machine ip> --port=<remote machine master port> --local-ip=<local pc ip> --setup-script=<dir/to/setup/script/in/remote/machine> --global-planner=<your global planner>  # You typically only needs `--local-ip`, and `--machine-ip`
```

Type `--help` for more information ([or see here](#Executables)).

## Public Interface

### Published Topics

-   TonyLocalPlanner/lookahead_point ([geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)): lookahead point calculated by local planner algorithm
-   TonyLocalPlanner/nearest_point ([geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)): nearest point calculated by local planner algorithm
-   TonyLocalPlanner/error ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/Header.html)): error message while running local planner algorithm, this topic publish message only when global plan doesn't satisfy precondition

### Subscribed Topic

-   odom ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)): car current pose so that it can calculate lookahead, nearest point, consequently, the control output to track the line

### Executables

-   add_sim_map.py: This executable helps create the required file to run the simulation.

        usage: add_sim_map.py [-h] [--map-dir MAP_DIR] [--map-name MAP_NAME]
                            [--robot ROBOT] [--init-pos INIT_POS] [--dry-run]

        Setup simulation environment for your map

        optional arguments:
          -h, --help           show this help message and exit
          --map-dir MAP_DIR    Directory to find scanned map picture
          --map-name MAP_NAME  The name of the map
          --robot ROBOT        The name of the robot, defaulted to environmental
                               variable ROBOT
          --init-pos INIT_POS  Initial position of the robot, the value must be comma
                               separated and enclosed with bracket, e.g. "[1,2,3]"
                               (with quotation mark)
          --dry-run            Print the result without executing the process

    example: `rosrun tony_local_planner add_sim_map.py --map-dir ~/catkin_ws/src/gyro/map/SPIL_4FLG --map-name SPIL_4FLG_1224 --robot spil-AMR-A-04L4 --init-pos "[28.841, 43.573 3.14]"`

    This will create working_field.png and working_field.yaml at `tony_local_planner/robots/spil-AMR-A-04L4`. You can use `--dry-run` in the above example to verify if the result is correct.

-   start_gain_tuning.sh: This executable launch gain tuning process

        Usage: rosrun tony_local_planner start_gain_tuning.sh [option...]

         -h, --help                     Show this help message and exit
         -l, --local-ip=LOCAL_IP        Set the ip address of your local pc
         -m, --machine-ip=MACHINE_IP    Set the ip address of the remote host
         -s, --setup-script=SCRIP_DIR   Set the directory of the setup script for remote launch process to execute during startup
         -g, --global-planner=PLANNER   Specify the global planner for plan generation, default global_planner/GlobalPlanner
         -p, --port=PORT_NUM            Specify the port number that the master is at, default 11311

### Parameters

-   ~max_vel_x (double, default: 0.5, unit: m/s):

    The maximum x velocity, the velocity here is defined wrt the line currently tracking

-   ~max_vel_y (double, default: 0.5, unit: m/s):

    The maximum y velocity, the velocity here is defined wrt the line currently tracking

-   ~max_vel_move_theta (double, default: 0.5, unit: rad/s):

    The absolute value of the maximum rotational velocity when moving

-   ~xy_goal_tolerance (double, default: 0.1, unit: meter):

    Local planner will stop move toward the goal if the distance between the car pose and goal pose is less than the tolerance

-   ~yaw_goal_tolerance (double, default: 0.05, unit: radian):

    Local planner will stop rotate toward the goal if the angle difference between car pose and goal pose is less than the tolerance

-   ~move_yaw_goal_tolerance (double, default: 0.0, unit: radian):

    Local planner will stop rotate toward the goal if the angle difference between car pose and goal pose is less than the tolerance while moving

-   ~max_lookahead_distance (double, default: 10.0, unit: meter):

    max lookahead distance

-   ~max_available_curvature (double, default: 10.0, unit: radian):

    max available curvature

-   ~off_track_condition (double, default: 1.0, unit: meter):

    If considered off-track, local planner will move back to the line first instead of moving toward the goal.

-   ~off_track_error_condition (double, default: 5.0, unit: meter):

    The maximum tolerable off-track distance, local planner will fail and report error if this value is exceeded.

-   ~transform_tolerance (double, default: 0.5, unit: second):

    transform_tolerance

-   ~/gain_tuning_helper/x_direction_pid (str):

    PID gain for tracking x direction distance, in the format: \[p, i, d], only available under tuning mode.

-   ~/gain_tuning_helper/y_direction_pid (str):

    PID gain for tracking y direction distance, in the format: \[p, i, d], only available under tuning mode.

-   ~/gain_tuning_helper/yaw_direction_pid (str):

    PID gain for tracking yaw difference, in the format: \[p, i, d], only available under tuning mode.

-   ~virtual_heading (double, default: 0.0, unit: radian):

    virtual_heading **(this is legacy value for backward compatibility and is deprecated)**

-   ~steady_state_criterion (int, default: 20):

    Steady state time represented using sampling time as time unit, e.g., for sampling time = 50ms, steady_state_criterion = 20 means that if the error is within certain tolerance for 50 \* 20 = 1000ms, then we consider the path tracking reaches steady state, i.e., MR moves to its destination.

## Debugging

Since local planner is a submodule of `move_base`, we need to make sure all parts of `move_base` node works fine, or are at least responsive.

-   Make sure `move_base` is still working:

    ```sh
    rosnode list | grep move_base # This should print /move_base if the node is still working
    rosnode ping /move_base # This checks whether /move_base becomes a zombie node or not
    ```

-   Check local planner velocity result:

    ```sh
    rostopic echo /nav_vel
    ```

-   Check odometry result since local planner depends on it:

    ```sh
    rostopic echo /odom
    ```

-   Check global plan local planner is following currently (@TODO)

-   Check local planner debug message by setting logger level to debug, set `logger` to `'ros.tony_local_planner'`, and `level` to `'Debug'`

    ```sh
    rosservice call /move_base/set_logger_level <tab> <tab>
    ```

## TODOs

-   Rename `amcl.yaml` and `move_base.yaml` to `amcl_sim.yaml` and `move_base_sim.yaml` to avoid misunderstanding and misuse
-   Investigate into the known bug in Unit Testing section
-   Find the other way to build unit test
-   Extend to multirobot situation
-   Add more tunable option for controller
-   Add documentation for roslaunch arg
-   More unit test
    -   test launch file validity with roslaunch --node and roslaunch --local
-   Make some script private (not visible when double tab)
-   Take costmap into consideration for velocity management

## Disclaimer

Please do not launch move_base via tony_local_planner in MR unless it is used for configuration, i.e., gain tuning or simulating. Since the move_base parameters in this package is configured in the way that only guarantees local planner will work, other things such as costmap, global planner, etc., are nearly not configured at all and remain their default values, which may not be what the real life situation would want.

In order to avoid misuses, the following table depicts the expected usage of the simulation.launch and gain_tuning.launch:

|                   | simulation | gain tuning | testing |
| :---------------: | :--------: | :---------: | :-----: |
|    locally (PC)   |      V     |     \*O     |    V    |
|    locally (MR)   |      X     |      V      |    X    |
| remotely (PC PoV) |      X     |      V      |    X    |

-   Gain tuning on your PC needs additional setup (coordinate transform, etc.) and is probably meaningless, but does no harm afterall

## TL, DR

-   PC run simulation: `roslaunch tony_local_planner simulation.launch robot:=<your robot>`
-   PC run unit tests:
    -   Only local planner: `rostest tony_local_planner local_planner_test.test robot:=<your robot>`
    -   Only controller: `rosrun tony_local_planner controller_test`
    -   Local planner + Global planner: `rostest tony_local_planner local_planner_test.test robot:=<your robot> global_planner:=<global planner>`
    -   Build and run all unit tests: `catkin run_tests tony_local_planner --no-deps`
-   PC run gain tuning locally: `roslaunch tony_local_planner gain_tuning.launch robot:=<your robot>` (still need additional set up)
-   MR run gain tuning locally: `roslaunch tony_local_planner gain_tuning.launch`
-   PC run GUIs while other nodes are launched remotely for gain tuning: `rosrun tony_local_planner start_gain_tuning.sh --local-ip=<your ip> --machine-ip=<machine ip> --setup-script=<dir/to/setup/script> --global-planner=<global planner> --port=<port of remote machine master>`

## Contact

Tony Guo (master branch) - tonyguo@gyro.com.tw

Jacky Tseng (asecl branch) - jacky.tseng@gyro.com.tw

### Reference

[stage_ros creating world](https://medium.com/@ivangavran/ros-creating-world-file-from-existing-yaml-5b553d31cc53#.88e8vmwuh)
