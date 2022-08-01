# tf_move_relative
- Relative motion between tf and tf with velocity planning function.

## Built with

- ROS Noetic under Ubuntu 20.04 LTS

- ROS Melodic under Ubuntu 18.04 LTS

------

## Getting Started

### Installation

- Need ROS tutorial package [tf2_learning](http://wiki.ros.org/tf2/Tutorials).

### Run

- Step1. Start tf_move_relative launch file.

  ``` $ roslaunch  tf_move_relative demo.launch ```

- Step2. Start tf_move_relative client cpp file and input go to relative tf position.

  ``` $ rosrun  tf_move_relative tf_move_relative_client ```

## tf_move_relative have Trapezoid and triangle velocity planning function.

- 1. Trapezoid velocity planning.

![image](https://github.com/qaz9517532846/tf_move_relative/blob/main/image/Trapezoid_vel_planning.png)

- 2. Triangle velocity planning

![image](https://github.com/qaz9517532846/tf_move_relative/blob/main/image/triangle_vel_planning.png)

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2020 ZM Robotics Software Laboratory.
