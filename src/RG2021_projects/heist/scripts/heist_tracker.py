#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import rogata_library as rgt
import numpy as np





def odom_callback( odom, argv):
    agent = argv[0]
    rogata = argv[1]
    pos = np.array([odom.pose.pose.position.x,
                   -odom.pose.pose.position.y])*100+np.array([500,500])
    rogata.set_pos(agent,pos)





if __name__ == '__main__':
    rospy.init_node("agent_tracker")
    try:
        rogata1 = rgt.rogata_helper()
        rogata2 = rgt.rogata_helper()
        rospy.Subscriber("guard/odom"  , Odometry, odom_callback,("guard_obj",rogata1))
        rospy.Subscriber("evader/odom", Odometry, odom_callback, ("evader_obj",rogata2))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
