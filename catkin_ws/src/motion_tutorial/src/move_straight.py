#!/usr/bin/env python3

from genpy import message
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import time
import math
from os import system
import sys

class MoveTurtlesim:
    def __init__(self, turtle_num, rate=20):
        print("Initializing Turtlesim Mover...")
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

    def rotate_to(self, deg_angle=0, speed=1, deg_threshold=1):
        print('\n-------------------')
        print("Turning in place...")

        angle = math.radians(deg_angle)
        rad_threshold = math.radians(deg_threshold)
        vel_msg = Twist()

        while not rospy.is_shutdown():
            if self.init_flag:
                break
            self.rate.sleep()

        t0 = self.theta
        
        def dir(ang=angle, t=t0):
            t_diff = ang-t
            if t_diff > math.pi:
                return -1
            elif t_diff < -math.pi:
                return 1
            elif t_diff > 0:
                return 1
            else:
                return -1

        set_dir = dir()
        while not rospy.is_shutdown():
        
            if set_dir > 0 and self.theta >= angle - rad_threshold:
                    break
            elif set_dir < 0 and self.theta <= angle + rad_threshold:
                    break
                
            # if abs(self.theta-angle) < rad_threshold:
            #     break
            
            vel_msg.angular.z = dir()*speed
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.angular.z = 0
        self.twist_pub.publish(vel_msg)

        print("Done turning")
        print(f"Current angle {math.degrees(self.theta)}")
        print('-------------------')

    def move_straight(self, speed, dist, dist_rad=0):
        print("Moving in a line...")
        vel_msg = Twist()

        while not rospy.is_shutdown():
            if self.init_flag:
                break
            self.rate.sleep()

        x0 = self.x
        y0 = self.y
        t0 = self.theta

        # print("\n---------------")
        # print("init x =", x0)
        # print("init y =", y0)
        # print("init t =", t0)
        # print("---------------\n")
        
        distance = lambda x,y:  math.sqrt((x-x0)**2 + (y-y0)**2)
        theta_dir = 1 if dist_rad > 0 else -1

        if dist_rad is not 0:
            print("Turning...")
            while not rospy.is_shutdown():
                if dist_rad > 0:
                    if self.theta >= t0 + dist_rad:
                        print("Correct t+")
                        break
                else:
                    if self.theta <= t0 + dist_rad:
                        print("Correct t-")
                        break

                vel_msg.angular.z = theta_dir*speed
                self.twist_pub.publish(vel_msg)
                self.rate.sleep()

            print("Done Turning")

            vel_msg.angular.z = 0
            self.twist_pub.publish(vel_msg)

        print("Moving in a line...")
        while not rospy.is_shutdown():
            if distance(self.x, self.y) >= dist:
                break

            vel_msg.linear.x = speed
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        self.twist_pub.publish(vel_msg)
        print("Finished")

    def reset_tbot(self):
        print("\nResetting...")
        rospy.wait_for_service('/reset')

        try:
            reset = rospy.ServiceProxy('/reset', Empty)
            resp = reset()
            print("Reset turtlebot")
        except rospy.ServiceException as e:
            print("Service call failed", e)


if __name__ == '__main__':
    system("clear")
    rospy.init_node('move_straight', anonymous=True)

    move = MoveTurtlesim(1)
    # move.move_straight(1, 5, -1.2)
    move.rotate_to(int(sys.argv[1]))

    # time.sleep(3)
    # move.reset_tbot()


        

        

