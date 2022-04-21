#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point, Twist

from math import atan2

 

x = 0

y = 0

theta = 0

odom_msg = 0

 

def newOdom(msg):

    global x

    global y

    global theta

    global odom_msg

 

    x = msg.pose.pose.position.x

    y = msg.pose.pose.position.y

 

    rot_q = msg.pose.pose.orientation

    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom_msg = 1

 

def position(x1, y1, x2, y2):

    return ((x1-x2)**2 +(y1-y2)**2)**0.5

 

print("start")

rospy.init_node("speed_controller")

 

#sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)

sub = rospy.Subscriber("/odom", Odometry, newOdom)

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

 

speed = Twist()

 

r = rospy.Rate(4)

 

#goal_list = [[19, 0], [0, 9], [-10, 0]]

#goal_list = [[0, -2]] #turn right

goal_list = [ [2, 0], [0, 2] ]

curr_point = goal_list.pop(0)

goal = Point()

goal.x = curr_point[0]

goal.y = curr_point[1]

 

while odom_msg <= 0:

    rospy.sleep(1)

#rospy.sleep(10)

 

print("controller start")

 

while not rospy.is_shutdown():

 

    inc_x = goal.x -x

    inc_y = goal.y -y

    print("odom x: %3.4f, y: %3.4f, theta: %3.4f" % (x,y,theta))

    angle_to_goal = atan2(inc_y, inc_x)

    if angle_to_goal > 0:

       sign = 1

    else:

       sign = -1

   

    #if position(goal.x, goal.y, x, y) <0.1:

    if position(goal.x, goal.y, x, y) <0.3:

               print("arrived.............................")

               if goal_list:

                    print("next pos")

                    curr_point = goal_list.pop(0)

                    goal.x = x + curr_point[0]

                    goal.y = y + curr_point[1]

 

               else:

                    print("end.............................")

                    speed.linear.x = 0.0

                    speed.angular.z = 0.0

                    print("controller publish")

                    pub.publish(speed)

                    rospy.sleep(100)

                    break

    #elif min(abs(angle_to_goal-theta), 6.28-abs(angle_to_goal-theta))  > 0.1 :

    elif min(abs(angle_to_goal-theta), 6.28-abs(angle_to_goal-theta))  > 0.5 :

              

               print("turn | ang diff: %3.4f" % (min(abs(angle_to_goal-theta), 6.28-abs(angle_to_goal-theta))))

               speed.linear.x = 0.0

               speed.angular.z = 0.8 * sign

    else:

               print("go straight | pos diff %3.4f" % (position(goal.x, goal.y, x, y)))

               speed.linear.x = 0.5

               speed.angular.z = 0.0

    print("controller publish")

    print("angle_to_goal: ", angle_to_goal, "theta: ", theta)

    pub.publish(speed)

 

    r.sleep()
