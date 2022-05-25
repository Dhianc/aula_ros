#!/usr/bin/env python

from numpy import rate
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import math
import time

x=0
y=0
yaw=0


def poseCallback(pose_message):
    global x, y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta


def move(velocity_publisher, speed, distance, is_forward):

    velocity_message = Twist()

    global x, y
    x0 = x
    y0 = y

    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

        distance_moved = distance_moved+abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)
        if not(distance_moved<distance):
            rospy.loginfo("reached")
            break

    #stop the robot when the distance is moved
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

def rotate(velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):

    velocity_message = Twist()

    angular_speed = math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    loop_rate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        if (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break

    #stop the robot when the distance is moved
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)

def go_to_goal(velocity_publisher, x_goal, y_goal):
    global x, y, yaw

    velocity_message = Twist()

    while True:
        k_linear = 0.5
        distance = abs(math.sqrt(((x_goal-x)**2) + ((y_goal-y)**2)))

        linear_speed = distance * k_linear

        k_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*k_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print('x=',x,', y=', y,', distance to goal: ', distance)
        if (distance< 0.01):
            break

def setDesiredOrientation(publisher, speed_in_degree, desired_angle_degree):
    relative_angle_radians = math.radians(desired_angle_degree)-yaw
    clockwise=0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print("relative_angle_radians: ", math.degrees(relative_angle_radians))
    print("desired_angle_degree: ", desired_angle_degree)
    rotate(publisher, speed_in_degree, math.degrees(abs(relative_angle_radians)), clockwise)

def spiral(velocity_publisher, rk, wk):
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)

    while((x<10.5) and (y<10.5)):
        rk=rk+1
        vel_msg.linear.x=rk
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x=0
        vel_msg.angular.y=0
        vel_msg.angular.z=wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()

    vel_msg.linear.x=0
    vel_msg.angular.z=0
    velocity_publisher.publish(vel_msg)

def gridClean(publisher):
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0

    go_to_goal(publisher, 1, 1)

    setDesiredOrientation(publisher, 30, math.radians(desired_pose.theta))

    for i in range(5):
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, False)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, False)
    pass

if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        position_topic = '/turtle1/pose'
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        #move(velocity_publisher, 10.0, 4.0, False)
        #rotate(velocity_publisher, 45, 45, False)
        #go_to_goal(velocity_publisher, 1.0, 1.0)
        #setDesiredOrientation(velocity_publisher, 30, 0)
        #spiral(velocity_publisher, 0, 2)
        gridClean(velocity_publisher)
    except rospy.ROSInterruptException:
        pass