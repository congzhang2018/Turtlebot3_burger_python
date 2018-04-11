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
        self.pillar_distance = 0
        self.pillar_angle = 0 
        self.line_distance = 0 
        self.line_angle = 0
        self.x = 0
        self.y = 0
        self.lidarData = []

        self.states = 1
        self.isSearching = True
        self.isFinish = False
        self.finishPosition = True
        self.finishAngluer = True
        self.finishLineFollowing = False

        self.move_cmd = Twist()

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
        Input: the desired position
        Robot will go to the desired position

        Return isFinish
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

            self.move_cmd.angular.z = angular_speed * path_angle-rotation
            self.move_cmd.linear.x = min(linear_speed * self.distance, 0.1)

            if self.move_cmd.angular.z > 0:
                self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
            else:
                self.self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)

            self.last_rotation = rotation
            self.cmd_pub.publish(self.move_cmd)
            isFinish = False
            return isFinish
        else
            rospy.loginfo("Stopping the robot...")
            self.cmd_pub.publish(Twist())
            isFinish = True
            return isFinish


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
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.5
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.5
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.5
            self.cmd_pub.publish(self.move_cmd)
            isFinish = False
            return isFinish
        else:
            rospy.loginfo("Stopping the robot...")
            self.cmd_pub.publish(Twist())
            isFinish = False
            return isFinish
        


    def GetDesiredAngle(self, angle):
        
        """
        Input: Desired turning angle in degrees
            passtive: turn right
            negetive: turn left
        Return: desired_angle in robot coordinate in degrees
        """

        [position, rotation] = self.GetOdom()
        current_angle = np.rad2deg(rotation)
        desired_anlge = current_angle + angle

        if desired_anlge > 180 
            desired_anlge = desired_anlge - 360
        else desired_anlge < -180
            desired_anlge = desired_anlge +360

        return desired_anlge


    def GetOdom(self):

        """
        Get tf massage and transfer to position and rotation

        Return position, yaw_angle
        """

        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def GetDesiredPosition(self, distance, angle):
        
        """
        Input: camera message
            distance:robot to the target
            angle: heading change angle
        return 
            x,y: location based on robot
        """
        [position, rotation] = self.GetOdom()
        x = position.x = distance*np.cos(angle)
        y = position.y = distance*np.sin(angle)

        return x, y



    def SearchingTarget(self):

        """
        Robot will turn right 60 degrees, for searching target
        """

        if self.isSearching
            self.desired_anlge = GetDesiredAngle(60) # turn right 60 degrees
            self.isSearching = False
        else    
            if self.isFinish
                self.cmd_pub.publish(Twist()) # stop robot
                self.isSearching = True
            else
                self.isFinish = self.FacetoPoint(self.desired_anlge)


    def ControlTurtlebot(self, v, z):
        
        """
        send heading velocity in m/s and the angle velocity in rads
        """
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = z

        self.cmd_pub.publisher(msg)


    def FollowLine(self):

        """
        Base on camera data moving to white line and following the white to the center
   
        """
        x, y = self.GetDesiredPosition(self.line_distance, self.line_angle)
        if self.isFinish
            self.cmd_pub.publish(Twist()) # stop robot
            self.finishPosition = True
            self.finishLineFollowing = True
        else
            self.isFinish = self.GotoPoint(x, y)




    def GotoPillar(self);

        """
        Base on camera data moving to center
   
        """
        x, y = self.GetDesiredPosition(self.pillar_distance, self.pillar_angle)
        if self.isFinish
            self.cmd_pub.publish(Twist()) # stop robot
            self.FinishPosition = True
        else
            self.isFinish = self.GotoPoint(x, y)


    def AvoidObject(self):

        """
        Base on lidar data avoid static object
   
        """
        for i in range(360):
            if i <= 15 or i > 335:
                if data.ranges[i] >= LIDAR_ERR:
                    scan_filter.append(data.ranges[i])

        minDist = min(scan_filter)
        if minDist < 0.5
            self.cmd_pub.publish(Twist())
        else
            self.states = 1
            return


    def FiniteMachine(self);

        """
        Running all processing  
   
        """
        self.AvoidObject()
        if self.states == 1
            self.SearchingTarget()
            if self.finishLineFollowing
                self.states = 3
        elif self.states == 2
            self.FollowLine()
        elif self.states == 3
            self.GotoPillar()
        else
            self.cmd_pub.publish(Twist())
            rospy.loginfo("Stopping the robot...")

    def CallbackCam(self, data):

        """
        Subscribe cammera data
   
        """
        self.pillar_distance = data.linear.x
        self.pillar_angle = data.linear.y 
        self.line_distance = data.angular.x 
        self.line_angle = data.angular.y



    def CallbackLider(self, data):

        """
        Subscribe lidar data
   
        """

        self.lidarData = data.ranges














if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            print(msg)
            GotoPoint()

    except:
        rospy.loginfo("shutdown program.")

