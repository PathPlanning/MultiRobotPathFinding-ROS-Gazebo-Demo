#!/usr/bin/env python
# coding=utf-8

import roslib
import rospy
import tf
import math
import numpy as np
import numpy.linalg as la
import sys
import os
import csv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import rospkg

current_pose = Pose2D()


class square:
    def driveForwardOdom(self, distance):
        cur_trans = []
        start_trans = []
        # wait for the listener to get the first message
        self.listener = tf.TransformListener()
        odom = '/robot_' + self.robot + '/odom'
        footprint = '/robot_' + self.robot + '/base_footprint'
        self.listener.waitForTransform(odom, footprint, rospy.Time(0), rospy.Duration(1.0))
        try:
            (start_trans, start_rot) = self.listener.lookupTransform(odom, footprint,
                                                                     rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print e.message

        # we will be sending commands of type "twist"
        base_cmd = Twist()
        # the command will be to go forward at 0.2 m/s
        base_cmd.linear.y = base_cmd.angular.z = 0
        base_cmd.linear.x = 0.2
        done = False
        while not done:
            # send the drive command
            self.p.publish(base_cmd)

            try:
                (cur_trans, cur_rot) = self.listener.lookupTransform(odom, footprint,
                                                                     rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            # see how far we've traveled

            a = np.asarray(start_trans)
            b = np.asarray(cur_trans)

            dist_moved = np.linalg.norm(a - b)

            ##  rospy.loginfo(dist_moved)

            if dist_moved >= distance:
                done = True

    def turnOdom(self, radians):

        if radians == 0:
            return
        start_yaw = 0
        cur_yaw = 0
        while radians < 0:
            radians = radians + 2 * math.pi
        while radians > 2 * math.pi:
            radians = radians - 2 * math.pi

        # wait for the listener to get the first message
        self.listener = tf.TransformListener()
        odom = '/robot_' + self.robot + '/odom'
        footprint = '/robot_' + self.robot + '/base_footprint'
        self.listener.waitForTransform(odom, footprint, rospy.Time(0), rospy.Duration(1.0))

        try:
            (start_trans, start_rot) = self.listener.lookupTransform(odom, footprint,
                                                                     rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(start_rot)
            start_yaw = euler[2]
            rospy.loginfo(start_yaw)
            if start_yaw < 0:
                start_yaw = 2 * math.pi - math.fabs(start_yaw)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRROR")
            pass

        # we will be sending commands of type "twist"
        base_cmd = Twist()
        # command will be to turn at 0.2 rad/s
        base_cmd.linear.x = base_cmd.linear.y = 0.0
        base_cmd.angular.z = 0.2

        done = False
        rospy.loginfo(start_yaw)
        rospy.loginfo(radians)
        rospy.sleep(3.0)
        while not done:
            # send the drive command
            self.p.publish(base_cmd)

            try:
                (cur_trans, cur_rot) = self.listener.lookupTransform(odom, footprint,
                                                                     rospy.Time(0))
                # see how far we've traveled
                euler = tf.transformations.euler_from_quaternion(cur_rot)
                cur_yaw = euler[2]
                if cur_yaw < 0:
                    cur_yaw = 2 * math.pi - math.fabs(cur_yaw)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRROR")
                pass

            angle_turned = cur_yaw - start_yaw

            if math.fabs(angle_turned) > radians:
                done = True

    def py_ang(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'    """
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)

    def readfile(self, robot, cellsize):
        commands = []

        ros_package = rospkg.RosPack()
        mover_directory = ros_package.get_path("pathplanning_mover2")
        directory = os.path.join(mover_directory, "src/commands/", self.robot + '.csv')
        print directory
        # Tут почему-то падает без исключительных ситуаций. Работало раньше.
        # Нужно детальное тестирование.
        try:
            with open(directory, "r") as f:
                reader = csv.reader(f)
                # Храним предыдущий вектор для того, чтобы вычислить угол.
                # Задаем изначальный, чтобы вычислить угол относительно оси X
                prev_row = ["0", "0", "1", "0"]
                print 'previous row' + prev_row
                for row in reader:
                    v1 = np.array([(int(prev_row[2]) - int(prev_row[0])) * cellsize,
                                   (int(prev_row[3]) - int(prev_row[1])) * cellsize])
                    v2 = np.array(
                        [(int(row[2]) - int(row[0])) * cellsize,
                         (int(row[3]) - int(row[1])) * cellsize])
                    alpha = self.py_ang(v1, v2)  # угол
                    vector_length = la.norm(v2)  # длина вектора передвижения
                    prev_row = row
                    commands.append([alpha, vector_length])
        except Exception as e:
            print e.message
        return commands

    def __init__(self, robot, cellsize):
        self.cellsize = float(cellsize)
        self.robot = str(robot)
        comlist = []
        """ This is the constructor of our class. """
        # register this function to be called on shutdown
        rospy.on_shutdown(self.cleanup)

        # publish to cmd_vel
        try:
            comlist = self.readfile(robot, self.cellsize)
        except Exception as e:
            print e.message
        link = '/robot_' + robot + '/mobile_base/commands/velocity'
        self.p = rospy.Publisher(link, Twist, queue_size="10")
        # give our node/publisher a bit of time to connect
        rospy.sleep(1.0)

        ##########################################################################################################
        #                          код не рассчитан на дубли ненулевых команд в одном логе.                      #
        #                       не должно быть одинаковых команд на перемещение из одних точек                   #
        ##########################################################################################################



        for com in comlist:
            print com
            rospy.loginfo(com)
            self.turnOdom(com[0])
            self.driveForwardOdom(com[1])

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.p.publish(twist)


if __name__ == "__main__":
    rospy.init_node('square' + str(sys.argv[1]))
    print ("Starting agent_ " + sys.argv[1])
    rospy.Rate(10)
    try:
        square(sys.argv[1], sys.argv[2])
    except Exception as e:
        print "need 2nd argument - cellsize!"
