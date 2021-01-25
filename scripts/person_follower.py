#!/usr/bin/env python3
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Proportional coefficient for linear velocity
KP_LIN = 0.3
# Proportional coefficient for angular velocity
KP_ANG = 0.01
# Angular velocity for turning corners
TURN_VEL = 200
# Safe distance from object
THRESHOLD = 0.7
# Greatest distance to start following the object at
DETECTION_LIMIT = 3.5

class PersonLoop(object):
    def __init__(self):
        rospy.init_node('person_follow')
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan = rospy.Subscriber("/scan", LaserScan, self.check_distance)
        self.bot_vel = Twist()

    def check_distance(self, data):
        closest_distance = np.amin(data.ranges)
        closest_angle = np.argmin(data.ranges)
        # if the bot is within the maximum follow distance
        if closest_distance < DETECTION_LIMIT:
            # if the bot is facing the object
            if (closest_angle < 90 or closest_angle > 270):
                self.bot_vel.linear.x = KP_LIN * max(closest_distance - THRESHOLD, 0)
            # if the object is to the left of the bot
            if (closest_angle < 180):
                self.bot_vel.angular.z = KP_ANG * (closest_angle)
            # if the object is to the right of the bot
            else:
                self.bot_vel.angular.z = KP_ANG * (closest_angle - 360)
        # if the bot is not within the maximum follow distance
        else:
            self.bot_vel.linear.x = 0

        self.cmd_vel.publish(self.bot_vel)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    myNode = PersonLoop()
    myNode.run()

