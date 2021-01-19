#!/usr/bin/env python3
""" This script receives ROS messages containing the 3D coordinates of a single point and prints them out """
import math
import rospy
from geometry_msgs.msg import Twist

class SquareLoop(object):
    def __init__(self):
        rospy.init_node('move_square')
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.bot_vel = Twist()

    def move_forward(self, speed):
        self.bot_vel.linear.x = speed
        self.cmd_vel.publish(self.bot_vel)

    def accelerate(self, speed):
        self.bot_vel.linear.x = self.bot_vel.linear.x + speed
        self.cmd_vel.publish(self.bot_vel)
    
    def stop(self):
        self.bot_vel.linear.x = 0
        self.bot_vel.angular.z = 0
        self.cmd_vel.publish(self.bot_vel)

    def turn_left(self, speed):
        self.bot_vel.angular.z = speed
        self.cmd_vel.publish(self.bot_vel)

    def run(self):
        r = rospy.Rate(5)
        ticks = 0
        while not rospy.is_shutdown():
            if (ticks == 0 or ticks == 70): 
                self.stop()
            # acceleration to avoid one of the wheels slipping
            elif (ticks >= 10 and ticks < 35):
                self.accelerate(0.01)
            elif (ticks >= 35 and ticks < 60):
                self.accelerate(-0.01)
            elif (ticks == 75):
                self.turn_left(math.pi/10)
            ticks = (ticks + 1) % 100
            r.sleep()


if __name__ == '__main__':
    myNode = SquareLoop()
    myNode.run()

