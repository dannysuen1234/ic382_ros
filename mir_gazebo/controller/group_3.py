#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sqrt

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def update_speed(goal):
    inc_x = goal.x - x
    inc_y = goal.y - y
    angle_to_goal = atan2(inc_y, inc_x)
    distance = sqrt(inc_x**2 + inc_y**2)

    # account for angle
    if angle_to_goal - theta > 0.1: #face to the destination
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    elif angle_to_goal - theta < -0.1: 
        speed.linear.x = 0.0
        speed.angular.z = -0.3
    else:
        # account for distance
        if distance > 0.1:
            speed.linear.x = 0.8
            speed.angular.z = 0.0
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
    pass


rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 0
goal.y = 8
count = 0


while not rospy.is_shutdown():

    inc_x = goal.x - x
    inc_y = goal.y - y
    distance = sqrt(inc_x**2 + inc_y**2)
    print(distance)
    update_speed(goal)

    #if abs(inc_x)<0.1 and abs(inc_y)<0.1:
    if distance<0.1:
	count = 1
	if count==1:
                print('~~~~~~~~~~~~~Reach Goal 1~~~~~~~~~~~~~~~~~~~~~')
		goal.x=goal.x+8
		count+=1
	 	inc_x = goal.x -x
    		inc_y = goal.y -y
  		angle_to_goal = atan2(inc_y, inc_x)

                while abs(inc_x)<0.1 and abs(inc_y)<0.1:
                    update_speed(angle_to_goal, theta)
                count+=1

	elif count==2:
                print('reached destination')
			
		

    


    pub.publish(speed)
    r.sleep() 
