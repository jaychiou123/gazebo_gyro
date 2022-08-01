#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import rospkg
import rogata_library as rgt
import numpy as np
import os


rospack    = rospkg.RosPack()
catch_path = rospack.get_path('heist')


filepath        = os.path.join(catch_path, 'maps/large_left_wall.npy')
left_wall       = np.load(filepath)
filepath        = os.path.join(catch_path, 'maps/right_column.npy')
right_column    = np.load(filepath)
filepath        = os.path.join(catch_path, 'maps/left_l.npy')
left_l          = np.load(filepath)
filepath        = os.path.join(catch_path, 'maps/first_bookcase.npy')
first_bookcase  = np.load(filepath)
filepath        = os.path.join(catch_path, 'maps/second_bookcase.npy')
second_bookcase = np.load(filepath)
filepath        = os.path.join(catch_path, 'maps/third_bookcase.npy')
third_bookcase  = np.load(filepath)
filepath        = os.path.join(catch_path, 'maps/fourth_bookcase.npy')
fourth_bookcase = np.load(filepath)
filepath        = os.path.join(catch_path, 'maps/middle_bar.npy')
middle_bar      = np.load(filepath)
walls_obj  = rgt.game_object('walls_obj', [right_column,left_wall,left_l,first_bookcase,second_bookcase,third_bookcase,fourth_bookcase,middle_bar], np.array([1]))



filepath   = os.path.join(catch_path, 'maps/entry.npy')
entry_point= np.load(filepath)
entry_obj  = rgt.game_object('entry_obj',[entry_point],np.array([1]))


filepath   = os.path.join(catch_path, 'maps/target.npy')
goal       = np.load(filepath)
goal_obj   = rgt.game_object('goal_obj',[goal],np.array([1]))

name      = "guard_obj"
cat_id    = 1
hit_box   = {"type":"rectangle","height":2,"width":2}
cat_obj   = rgt.dynamic_object(name,hit_box,cat_id)

name      = "evader_obj"
mouse_id  = 1
hit_box   = {"type":"rectangle","height":2,"width":2}
mouse_obj = rgt.dynamic_object(name,hit_box,mouse_id)


if __name__ == '__main__':
    rospy.init_node("rogata_engine")
    try:
        example_scene = rgt.scene([walls_obj,cat_obj,mouse_obj,entry_obj, goal_obj])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
