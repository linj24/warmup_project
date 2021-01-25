#!/usr/bin/env python3
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

KP_LIN = 0.3 
KP_ANG = 0.01
TURN_VEL = 200 
DIST_TO_ANG = 90
DETECTION_LIMIT = 3.5
THRESHOLD = 0.7 

class PersonLoop(object):
    def __init__(self):
        rospy.init_node('person_follow')
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan = rospy.Subscriber("/scan", LaserScan, self.check_distance)
        self.bot_vel = Twist()

    def check_distance(self, data):
        closest_distance = np.amin(data.ranges)
        closest_angle = np.argmin(data.ranges)
        if closest_distance < DETECTION_LIMIT:
            if (closest_angle < 90 or closest_angle > 270):
                self.bot_vel.linear.x = KP_LIN * max(closest_distance - THRESHOLD, 0)
            if (closest_angle < 180):
                self.bot_vel.angular.z = KP_ANG * (closest_angle)
            else:
                self.bot_vel.angular.z = KP_ANG * (closest_angle - 360)
        else:
            self.bot_vel.linear.x = 0

        self.cmd_vel.publish(self.bot_vel)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    myNode = PersonLoop()
    myNode.run()

