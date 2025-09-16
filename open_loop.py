#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()


    def run(self):
        vel = Twist()
        for j in range(4):
            rospy.loginfo("traveling")
            vel.linear.x = 0
            vel.angular.z = 0
            for i in range(165):
                vel.linear.x += 0.0015
                self.vel_pub.publish(vel)
                self.rate.sleep()
            for i in range(165):
                vel.linear.x -= 0.0015
                self.vel_pub.publish(vel)
                self.rate.sleep()
            vel.linear.x = 0
            self.vel_pub.publish(vel)

            rospy.loginfo("turning")
        
            vel.linear.x = 0
            vel.angular.z = pi/2
            for i in range(11):
                self.vel_pub.publish(vel)
                self.rate.sleep()
            vel.angular.z = 0
            self.vel_pub.publish(vel)
        

if __name__ == '__main__':
    try:
        tb = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
