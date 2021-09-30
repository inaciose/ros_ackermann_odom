#!/usr/bin/env python3

'''
 Estimates odometry using only velocity commands from twist 
 messages without the use of wheel sensors.

 based on: https://medium.com/@waleedmansoor/how-i-built-ros-odometry-for-differential-drive-vehicle-without-encoder-c9f73fe63d87
'''


import rospy
from math import tan, cos, sin
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
import tf

import math

# variables
speed_linear = 0
speed_angular = 0

init_x_pos = 0.0
init_y_pos = 0.0
init_yaw = 0.0

def twist_callback(data):
    
    global speed_linear
    global speed_angular

    # Get speed and steering values
    speed_linear = data.linear.x
    speed_angular = data.angular.z

    # calculate average velocities ?

def main():

    global speed_linear
    global speed_angular
    global init_x_pos
    global init_y_pos
    global init_yaw

    #wheelbase = 0.2

    # Set up node
    rospy.init_node('ackermann_odom', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Publisher
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
    tf_br = tf.TransformBroadcaster()

    # Subscriptions
    rospy.Subscriber("cmd_vel", Twist, twist_callback)

    odom_msg = Odometry()

    last_stamp = rospy.Time.now()

    x_ = 0.0
    y_ = 0.0
    yaw_ = 0.0

    while not rospy.is_shutdown():

        #radius = speed_linear / speed_angular
        #steering_angle = math.atan(wheelbase / radius)


        # Delta time
        dt = rospy.Time.now() - last_stamp

        # Get yaw from angular velocity of IMU
        yaw_ += speed_angular * dt.to_sec()

        # Calculate speed in x and y direction
        x_dot = speed_linear * cos(yaw_)
        y_dot = speed_linear * sin(yaw_)

        # Calculate x and y position
        x_ += x_dot * dt.to_sec()
        y_ += y_dot * dt.to_sec()

        # Save timestamp
        last_stamp = rospy.Time.now()

        # Make odometry msg
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = "base_link"

        # Position and orientation
        odom_msg.pose.pose.position.x = x_
        odom_msg.pose.pose.position.y = y_
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = sin(yaw_/2.0)
        odom_msg.pose.pose.orientation.w = cos(yaw_/2.0)

        # Uncertainty in position
        odom_msg.pose.covariance[0] = 0.5 # <x
        odom_msg.pose.covariance[7]  = 0.5 # <y
        odom_msg.pose.covariance[35] = 0.4 # <yaw

        # Velocity
        odom_msg.twist.twist.linear.x = speed_linear
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = speed_angular
        odom_msg.twist.covariance[0] = 0.5 # <x
        odom_msg.twist.covariance[7]  = 0.5 # <y
        odom_msg.twist.covariance[35] = 0.4 # <yaw

        # Publish msgs
        odom_pub.publish(odom_msg)

        # Odom transform to
        tf_br.sendTransform((x_, y_, 0.0), (0.0, 0.0, sin(yaw_/2.0), cos(yaw_/2.0)), rospy.Time.now() , "base_link", "odom")
        tf_br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now() , "odom", "map")

        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

