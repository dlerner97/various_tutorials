#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import time
import math
from os import system
import sys

class MoveToGoal:
    def __init__(self, turtle_num, rate=20):
        print("Initializing Turtlesim Goal Pos Controller...\n")
        self.turtle_num = str(turtle_num)
        self.namespace = '/turtle'+self.turtle_num
        self.init_flag = False
        self.rate = rospy.Rate(rate)

        def set_pose(msg):
            self.x = msg.x
            self.y = msg.y
            self.theta = msg.theta
            self.linear_velocity = msg.linear_velocity
            self.angular_velocity = msg.angular_velocity
            self.init_flag = True

        self.pose_sub = rospy.Subscriber(self.namespace+'/pose', Pose, set_pose)
        self.twist_pub = rospy.Publisher(self.namespace+'/cmd_vel', Twist, queue_size=10)

    def move_to(self, goal, k_lin=.5, k_ang=4, dist_thresh=.1):
        rospy.loginfo('-------------------')
        
        vel_msg = Twist()
        dist = lambda x, y: (math.sqrt((x-goal[0])**2 + (y-goal[1])**2), math.atan2(goal[1]-y, goal[0]-x))

        while not rospy.is_shutdown():
            if self.init_flag:
                break
            self.rate.sleep()

        rospy.loginfo("Heading to goal position...")

        while not rospy.is_shutdown():
            curr_dist = dist(self.x, self.y)
            # rospy.loginfo(curr_dist)

            if curr_dist[0] < dist_thresh:
                rospy.loginfo("Arrived at destination")
                break

            lin_spd = k_lin*curr_dist[0]
            ang_spd = k_ang*(curr_dist[1]-self.theta)

            vel_msg.linear.x = lin_spd
            vel_msg.angular.z = ang_spd

            self.twist_pub.publish(vel_msg)

            print(f"x={self.x}, y={self.y}, dist={curr_dist}")

            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.twist_pub.publish(vel_msg)
        rospy.loginfo('-------------------')

if __name__ == '__main__':
    system("clear")
    rospy.init_node('move_to_goal', anonymous=True)
    move = MoveToGoal(1)
    move.move_to((float(sys.argv[1]), float(sys.argv[2])))


