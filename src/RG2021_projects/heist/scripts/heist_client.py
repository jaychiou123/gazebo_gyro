#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import rogata_library as rgt
import numpy as np


def visibility(guard,thief,wall_objects,max_seeing_distance):
    distance   = np.linalg.norm(thief-guard)
    direction  = (thief-guard)/distance
    direction  = np.arctan2(direction[1],direction[0])

    min_intersect = guard + max_seeing_distance * np.array([np.cos(direction),np.sin(direction)])

    for walls in wall_objects:

        intersection = rogata.intersect(walls,guard,direction,max_seeing_distance)
        if np.linalg.norm(intersection-guard) <= np.linalg.norm(min_intersect-guard):
            min_intersect = intersection

    if np.linalg.norm(min_intersect-guard) >= distance:
        return 1
    else:
        return 0



if __name__ == '__main__':
    rospy.init_node("Refree")
    rogata = rgt.rogata_helper()

    game_state = Int32()
    game_state = 0

    has_bounty = 0
    rate       = rospy.Rate(10)  # 10hz
    pub        = rospy.Publisher("game_state", Int32, queue_size=10)

    try:
        while not rospy.is_shutdown():
            guard_pos    = rogata.get_pos("guard_obj")
            evader_pos   = rogata.get_pos("evader_obj")

            evader_visible = visibility(guard_pos,evader_pos,["walls_obj"],1000)
            if  rogata.inside("goal_obj",evader_pos):
                has_bounty = 1
                print("Got the Bounty!")
            if  rogata.inside("entry_obj",evader_pos) and has_bounty:
                game_state = 1
                print ("The Evader Wins")
            if evader_visible:
                game_state = -1
                print("The Guard Wins!")


            pub.publish(game_state)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
