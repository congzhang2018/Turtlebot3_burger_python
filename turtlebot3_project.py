#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan

import time
import numpy as np
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion


LIDAR_ERR = 0.05
linear_speed = 0.5
angular_speed = 0.5

class Turtlebot3

    def __init__(self):

        # initial class values

        self.desired_anlge = 0
        self.yaw_radians = 0
        self.last_rotation = 0
        self.x = 0
        self.y = 0

        self.states = 1
        self.isSearching = True

        # initial command publisher
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)



        # initial odom_frame tf_listener
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

    

    def GotoPoint(self, x, y):

        """
        Input: the desired posistion
        Robot will go to the desired posistion
        """

        (position, rotation) = self.GetOdom()
        goal_distance = sqrt(pow(x - position.x, 2) + pow(y - position.y, 2))
        path_angle = atan2(y, x)
        self.distance = goal_distance

        if self.distance > 0.05:
            path_angle = atan2(y - y_start, x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if y < 0 and y_start < y:
                    path_angle = -2*pi + path_angle
                elif y >= 0 and y_start > y:
                    path_angle = 2*pi + path_angle
            if self.last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif self.last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation

            move_cmd.angular.z = angular_speed * path_angle-rotation
            move_cmd.linear.x = min(linear_speed * self.distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            self.last_rotation = rotation
            self.cmd_pub.publish(move_cmd)
        else
            rospy.loginfo("Stopping the robot...")
            self.cmd_pub.publish(Twist())
    


    def FacetoPoint(self, z):

        """
        Input: the desired yaw angle
        Robot will face to desired specific direction
        """
    
        goal_z = np.deg2rad(z)
        (position, rotation) = self.GetOdom()
        self.error_angle = abs(rotation - goal_z)
        if self.error_angle > 0.05:
            (position, rotation) = self.GetOdom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_pub.publish(move_cmd)
        else:
            rospy.loginfo("Stopping the robot...")
            self.cmd_pub.publish(Twist())
        


    def Searching_target(self):

        """
        Robot will turn right 60 degrees, for searching target
        """

        if self.isSearching
            self.desired_anlge = Get_desired_angle(60) # turn right 60 degrees
            self.isSearching = False
        else    
            if self.error_angle < 0.05
                self.cmd_pub.publish(Twist()) # stop robot
                self.isSearching = True
            else
                FacetoPoint(self.desired_anlge)



    def Get_desired_angle(self, angle):
        """
        Input: Desired turning angle in degrees
            passtive: turn right
            negetive: turn left
        Return: desired_angle in robot coordinate in degrees
        """
        [posistion, rotation] = self.GetOdom()
        current_angle = np.rad2deg(rotation)
        desired_anlge = current_angle + angle

        if desired_anlge > 180 
            desired_anlge = desired_anlge - 360
        else desired_anlge < -180
            desired_anlge = desired_anlge +360

        return desired_anlge


    def GetOdom(self):

        """
        Get tf massage and transfer to posistion and rotation

        Return posistion, yaw_angle
        """

        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def FollowLine(self):

        """
        Base on camera data moving to white line and following the white to the center
   
        """


    def GotoPillar(self);

        """
        Base on camera data moving to center
   
        """


    def AvoidObject(self):

        """
        Base on lidar data avoid static object
   
        """

    
    def FiniteMachine(self);

        """
        Running all processing  
   
        """


    def CallbackCam(self, data):

        """
        Subscribe cammera data
   
        """

        [posistion, rotation] = self.GetOdom()


    def CallbackLider(self, data):

        """
        Subscribe lidar data
   
        """

        self.scan_filter = []

        for i in range(360):
            if i <= 15 or i > 335:
                if data.ranges[i] >= LIDAR_ERR:
                    self.scan_filter.append(data.ranges[i])












if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            print(msg)
            GotoPoint()

    except:
        rospy.loginfo("shutdown program.")

