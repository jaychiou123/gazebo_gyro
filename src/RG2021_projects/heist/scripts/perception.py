#! /usr/bin/env python
import rospy
from math import *
import sys
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf

class Perception:

    def __init__(self):
        self.source_topic = sys.argv[1]
        self.published_topic = sys.argv[2]
        self.a1 = float(sys.argv[3]) 
        self.a2 = pi/18
        self.a3 = float(sys.argv[4])
        self.a4 = float(sys.argv[5])

        self.last_odom = None
        self.pose = [0.0,0.0,0.0]
        self.percepted_odom = Odometry()
        self.imu = Imu()
        self.diff_odom = Odometry()

        rate = rospy.Rate(40.0)  # 0.5hz
        rospy.Subscriber(self.source_topic, Odometry, self.source_callback)
        rospy.Subscriber("imu_data", Imu, self.imu_callback)
        rospy.Subscriber("odometry/filtered", Odometry, self.diff_callback)

        self.pub = rospy.Publisher(self.published_topic, Odometry, queue_size=10)
        self.pub1 = rospy.Publisher("imu_data_cov", Imu, queue_size=10)
        self.pub2 = rospy.Publisher("odom_diff", Odometry, queue_size=10)

        while not rospy.is_shutdown():
            self.pub.publish(self.percepted_odom)
            self.pub1.publish(self.imu)
            self.pub2.publish(self.diff_odom)
            rate.sleep()

    def diff_callback(self, odom_filtered):
        self.diff_odom.header = odom_filtered.header
        self.diff_odom.child_frame_id = odom_filtered.child_frame_id
        self.diff_odom.pose.pose.position.x = odom_filtered.pose.pose.position.x - self.last_odom.pose.pose.position.x 
        self.diff_odom.pose.pose.position.y = odom_filtered.pose.pose.position.y - self.last_odom.pose.pose.position.y 

    def imu_callback(self, imu):
        self.imu.header = imu.header
        self.imu.orientation = imu.orientation
        self.imu.angular_velocity = imu.angular_velocity
        self.imu.linear_acceleration = imu.linear_acceleration
        # self.imu.orientation_covariance = [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0]
        # self.imu.angular_velocity_covariance = [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0000008]  #pow(0.0475034, 2)
        # self.imu.linear_acceleration_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 100.0]   # pow(0.000209338, 2)   pow(0.000254688, 2)
        self.imu.orientation_covariance = [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0]
        self.imu.angular_velocity_covariance = [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.000000008 ]  # 
        self.imu.linear_acceleration_covariance = [0.00000001, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.0, 100.0]   
        
    def source_callback(self, odom):
        q = [ odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w ]
        (r, p, theta2) = tf.transformations.euler_from_quaternion(q)
 
        if(self.last_odom == None):
            self.last_odom = odom
            self.pose[0] = odom.pose.pose.position.x
            self.pose[1] = odom.pose.pose.position.y
            self.pose[2] = theta2
        else:
            dx = odom.pose.pose.position.x - self.last_odom.pose.pose.position.x
            dy = odom.pose.pose.position.y - self.last_odom.pose.pose.position.y
            trans = sqrt(dx*dx + dy*dy)
            q = [ self.last_odom.pose.pose.orientation.x,
                    self.last_odom.pose.pose.orientation.y,
                    self.last_odom.pose.pose.orientation.z,
                    self.last_odom.pose.pose.orientation.w ]
            (r,p, theta1) = tf.transformations.euler_from_quaternion(q)
            rot1 = atan2(dy, dx) - theta1
            rot2 = theta2-theta1-rot1
    
            sd_rot1 = self.a1*abs(rot1) + self.a2*trans
            sd_rot2 = self.a1*abs(rot2) + self.a2*trans
            sd_trans = self.a3*trans + self.a4*(abs(rot1) + abs(rot2))
    
            trans +=  np.random.normal(0,sd_trans*sd_trans)
            rot1 += np.random.normal(0, sd_rot1*sd_rot1)
            rot2 += np.random.normal(0, sd_rot2*sd_rot2)
    
            self.pose[0] += trans*cos(theta1+rot1)
            self.pose[1] += trans*sin(theta1+rot1)
            self.pose[2] +=  rot1 + rot2
            
        self.percepted_odom.header = odom.header
        self.percepted_odom.child_frame_id = odom.child_frame_id

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.pose[2])
        
        self.percepted_odom.pose.pose.position.x = self.pose[0]
        self.percepted_odom.pose.pose.position.y = self.pose[1]
        self.percepted_odom.pose.pose.orientation.x = quaternion[0]
        self.percepted_odom.pose.pose.orientation.y = quaternion[1]
        self.percepted_odom.pose.pose.orientation.z = quaternion[2]
        self.percepted_odom.pose.pose.orientation.w = quaternion[3]
        self.percepted_odom.twist.twist.linear.x = odom.twist.twist.linear.x + np.random.normal(0.00, 0.01) 
        self.percepted_odom.twist.twist.linear.y = odom.twist.twist.linear.y + np.random.normal(0.00, 0.01) 
        self.percepted_odom.twist.twist.angular.z = odom.twist.twist.angular.z + np.random.normal(0.00, 0.01) 
        self.percepted_odom.pose.covariance = odom.pose.covariance            # 0.001
        self.percepted_odom.twist.covariance = odom.pose.covariance
        self.percepted_odom.twist.covariance = list(self.percepted_odom.twist.covariance)
        self.percepted_odom.twist.covariance[35] = 0.00001
        self.percepted_odom.twist.covariance = tuple(self.percepted_odom.twist.covariance)
        self.last_odom = odom

        
if __name__ == '__main__':
    rospy.init_node("Perception")
    try:
        node = Perception()
    except rospy.ROSInterruptException:
        pass    
    