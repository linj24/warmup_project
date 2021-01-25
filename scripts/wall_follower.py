#!/usr/bin/env python3
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

KP_LIN = 0.5
KP_ANG = 0.01
TURN_VEL = 200
DIST_TO_ANG = 90
THRESHOLD = 0.7

class WallLoop(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan = rospy.Subscriber("/scan", LaserScan, self.check_distance)
        self.bot_vel = Twist()
        self.wall_side = 1 # -1 for wall on left, 1 for wall on right

    def check_distance(self, data):
        closest_distance = np.amin(data.ranges)
        closest_angle = np.argmin(data.ranges)
        self.bot_vel.linear.x = KP_LIN * max(closest_distance, THRESHOLD)
        if (closest_angle < 30 or closest_angle > 330):
            self.bot_vel.linear.x = 0
            if self.wall_side == 1:
                self.bot_vel.angular.z = KP_ANG * TURN_VEL
            else:
                self.bot_vel.angular.z = KP_ANG * TURN_VEL
        elif (closest_angle < 180):
            self.bot_vel.angular.z = KP_ANG * (closest_angle - 90 - 
                                               (DIST_TO_ANG * (THRESHOLD - closest_distance)))
            self.wall_side = -1
        else:
            self.bot_vel.angular.z = KP_ANG * (closest_angle - 270 +
                                               (DIST_TO_ANG * (THRESHOLD - closest_distance)))
            self.wall_side = 1

        self.cmd_vel.publish(self.bot_vel)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    myNode = WallLoop()
    myNode.run()

