#!/usr/bin/env python3
import rospy
import tf
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Twist, Pose2D
from nav_msgs.msg import Odometry

class LeftWallFollowerTF:
    def __init__(self):
        rospy.init_node('left_wall_follower')
        self.cmd_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tf_listener=tf.TransformListener()
        #subscribe to lidar
        rospy.Subscriber('scan', LaserScan, self.scan_cb)
        rospy.loginfo("LeftWallFollower initialized")
        self.rate = rospy.Rate(10)
        self.c = 0 #keeps track of how much the robot has turned
        self.nl = 0 #keeps track if there are no left points twice in a row
        self.pose = Pose2D()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.dist_from_goal_x = 1.0
        self.dist_from_goal_y = 1.0

    def scan_cb(self,scan):
        fpts=[]
        pts=[]
        #TASK 1 gather points in 1.52-1.62 rad(left) window
        rospy.loginfo("task 1")
        for i,r in enumerate(scan.ranges):
            if r == float('inf') or r==0.0:
                continue
            angle=scan.angle_min + i * scan.angle_increment
            if 1.52 <= angle <= 1.62:
                x_s = r * math.cos(angle)
                y_s = r * math.sin(angle) 
                p=PointStamped()
                p.header=scan.header
                p.header.frame_id = scan.header.frame_id
                p.point.x = x_s
                p.point.y = y_s
                p.point.z = 0.0 
                pts.append(p)
        
        #gather points in front of the robot
        for i,r in enumerate(scan.ranges):
            if r == float('inf') or r==0.0:
                continue
            angle=scan.angle_min + i * scan.angle_increment
            if -0.1 <= angle <= 0.1:
                x_s = r * math.cos(angle)
                y_s = r * math.sin(angle) 
                p=PointStamped()
                p.header=scan.header
                p.header.frame_id = scan.header.frame_id
                p.point.x = x_s
                p.point.y = y_s
                p.point.z = 0.0 
                fpts.append(p)

        
        #TASK 2 Transform each point into 'left_limit', compute distance
        rospy.loginfo("task2")
        dist=[]
        for p in pts:
            try:
                self.tf_listener.waitForTransform('left_limit', p.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                p_out = self.tf_listener.transformPoint('left_limit', p)
                dx=p_out.point.x
                dy=p_out.point.y
                dist.append(math.hypot(dx,dy))
            except(tf.LookupException, tf.ExtrapolationException):
                rospy.logwarn_throttle(5,"TF to left limit not available")
                continue

        fdist=[]
        for p in fpts:
            try:
                self.tf_listener.waitForTransform('base_footprint', p.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                p_out = self.tf_listener.transformPoint('base_footprint', p)
                dx=p_out.point.x
                dy=p_out.point.y
                fdist.append(math.hypot(dx,dy))
            except(tf.LookupException, tf.ExtrapolationException):
                rospy.logwarn_throttle(5,"TF to base_footprint not available")
                continue
            
        
        #Task 3 Behavior based on min_dist and front_scan
        rospy.loginfo("task3")
        front_scan=0.4 #if there is no points in front of robot,this has the first if statement return false
        if fdist:
            front_scan=min(fdist)
        
        cmd=Twist()

        #turn if there is an obstacle closer than 0.3m
        if (self.dist_from_goal_x <= 0.3 and self.dist_from_goal_y <= 0.3):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif front_scan<0.2: 
            rospy.loginfo("obstacle detected at %.4fm, now turning", front_scan)
            cmd.linear.x = 0
            cmd.angular.z = -math.pi/2
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        #turn left if there are no points on the left side twice in a row
        elif not dist:
            if self.nl == 1:
                rospy.loginfo("no obstacle detected, now turning")
                cmd.linear.x = 0.05
                cmd.angular.z = math.pi/2
                self.cmd_pub.publish(cmd)
                self.rate.sleep()
            else:
                self.nl = 1
        #Otherwise move forward at 0.1 m/s 
        else:
            min_dist=min(dist)
            self.nl = 0
            rospy.loginfo("distance: %.4fm", min_dist)

            goal = [7.7,0]
            self.dist_from_goal_x = abs(goal[0] - self.pose.x) 
            self.dist_from_goal_y = abs(goal[1] - self.pose.y)
            rospy.loginfo("Distance from goal: " + str(self.dist_from_goal_x) + ", " + str(self.dist_from_goal_y))
            
            #to keep the appropriate distance from the wall
            #if the closest wall reading is below 0.1, steer right
            if min_dist < 0.1 and self.c < 5:
                rospy.loginfo("turning right")
                cmd.angular.z = -0.5
                cmd.linear.x = 0.1
                if self.c < 5:
                    self.c +=1

            #if the closest wall reading is above 0.2, steer left
            elif min_dist >0.2 and self.c > -5:
                if min_dist > 0.5 and min_dist < 8: #hard left turn
                    rospy.loginfo("no obstacle detected, now turning")
                    cmd.linear.x = 0.05
                    cmd.angular.z = math.pi/2
                    self.cmd_pub.publish(cmd)
                    self.rate.sleep()
                else:
                    rospy.loginfo("turning left")
                    cmd.angular.z = 0.5
                    cmd.linear.x = 0.1
                    if self.c > -5:
                        self.c -=1

            #if between 0.1 and 0.2, go straight
            else:
                rospy.loginfo("going straight")
                cmd.linear.x = 0.1
                #cmd.angular.z =0
                self.c = 0
                cmd.angular.z = self.c * 0.5
                #turns the robot the opposite amount it has previously turned
        self.cmd_pub.publish(cmd)
        
    def odom_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y 

    def run(self):
        rospy.spin()

if __name__=='__main__':
    try:
        rospy.init_node('left_wall_follower')  
        node = LeftWallFollowerTF()
        node.run()
    except rospy.ROSInterruptException:
        pass
        
        

