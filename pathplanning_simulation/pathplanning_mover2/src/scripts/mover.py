#!/usr/bin/env python
# coding=utf-8
import roslib
import rospy
import math
import tf
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty as EmptyMsg
import sys
import os
import rospkg
import csv
import numpy as np
import numpy.linalg as la
from numpy import ones, vstack
from numpy.linalg import lstsq
import timeit


class mover:
    def turn_robot_around(self, yaw):
        quaternion = tf.transformations.turn_robot_around(0, 0, -yaw)
        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]

    def move_forward(self, array_of_coords, time):
        if (len(array_of_coords) == 0):
            return
        for element in array_of_coords:
            exectime = timeit.default_timer()
            print element
            self.pose.position.x = element[0]
            self.pose.position.y = element[1]
            self.state.model_name = "robot_" + self.robot
            self.state.pose = self.pose
            # rospy.loginfo("Moving robot")
            try:
                ret = self.g_set_state(self.state)
                print ret.status_message
            except Exception, e:
                rospy.logerr('Error on calling service: %s', str(e))
            rospy.loginfo("Unpausing physics")
            # try:
            #     self.g_unpause()
            # except Exception, e:
            #     self.rospy.logerr('Error on calling service: %s', str(e))
            self.loc = PoseWithCovarianceStamped()
            self.loc.pose.pose = self.pose
            self.loc.header.frame_id = "/map"
            rospy.loginfo("Adjusting localization")
            self.pub.publish(self.loc)
            rospy.sleep(time / (len(array_of_coords) - (timeit.default_timer() - exectime)))
            print (time / (len(array_of_coords) - (timeit.default_timer() - exectime)))
            print time

    def calculate_coords(self, command):
        # находим k и x для формулы линейного уравнения y = kx +b
        points = [(command[2], command[3]), (command[4], command[5])]
        x_coords, y_coords = zip(*points)
        A = vstack([x_coords, ones(len(x_coords))]).T
        m, c = lstsq(A, y_coords)[0]
        # если стартовые позиции численно больше конечных - нам нужно двигаться влево или вниз
        if command[2] > command[4]:
            deltax = -0.02
        elif command[2] < command[4]:
            deltax = 0.02
        # delta нельзя присвоить 0, тк итератор должен быть ненулевым числом
        else:
            deltax = 0.02
        if command[3] > command[5]:
            deltay = -0.02
        elif command[3] < command[5]:
            deltay = 0.02
        else:
            deltay = 0.02
        array_of_coords = []
        x = command[2]
        y = command[3]
        # self.calculate_angle(command[0])
        # пробегаем по x от стартовых до конечных позиций, с шагом deltax
        for x in np.arange(command[2], command[4], deltax):
            print x
            y = m * x + c
            x += deltax
            array_of_coords.append([x, y])
        # если startx примерно равна funish x:
        if (command[4] - 0.005) < x < (command[4] + 0.005):
            # пробегаем по y, инкрементируя его на дельту. тк
            # x не изменяется - просто изменяем y
            for y in np.arange(y, command[5], deltay):
                y += deltay
                array_of_coords.append([x, y])
        print array_of_coords
        return array_of_coords

    # def calculate_angle(self, radians):
    #     if radians == 0:
    #         return
    #     else:
    #         rads =self.angle_to_quaternion(radians)
    #         return rads


    def py_ang(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'    """
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)

    def readfile(self):
        commands = []
        ros_package = rospkg.RosPack()
        mover_directory = ros_package.get_path("pathplanning_mover2")
        directory = os.path.join(mover_directory, "src/commands/", self.robot + '.csv')
        print directory
        try:
            with open(directory, "r") as f:
                reader = csv.reader(f)
                print "readed"
                for row in reader:
                    v1 = np.array([int(row[2]) - int(row[0]) ,
                                   int(row[3]) - int(row[1])])
                    # negative x axis
                    v2 = np.array([-1 , 0])
                    alpha = self.py_ang(v1, v2)  # угол
                    vector_length = la.norm(v2)  # длина вектора передвижения
                    commands.append([alpha, vector_length,
                                     int(row[0]) * self.cellsize + self.cellsize / 2,
                                     int(row[1]) * self.cellsize + self.cellsize / 2,
                                     int(row[2]) * self.cellsize + self.cellsize / 2,
                                     int(row[3]) * self.cellsize + self.cellsize / 2, float(row[4])])
        except Exception as e:
            print e.message
        return commands

    def __init__(self, robot, cellsize):

        self.g_pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptySrv)
        self.g_unpause = rospy.ServiceProxy("/gazebo/unpause_physics", EmptySrv)
        self.g_set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, latch=True)
        sim = rospy.get_param('/use_sim_time')
        self.pose = Pose()
        self.state = ModelState()
        self.robot = str(robot)
        print cellsize
        self.cellsize = float(cellsize)
        self.time = 0.0
        for command in self.readfile():
            print 'command: ' + str(command)
            self.time = command[6]
            coords = self.calculate_coords(command)
            print 'coordinates: ' + str(coords)
            if len(coords) != 0:
                self.turn_robot_around(command[0])
                self.move_forward(coords, self.time)
            else:
                rospy.sleep(command[6])
        if sim is True:
            rospy.loginfo('Simulation is completely ready, publishing to /sim_init topic')

            pub = rospy.Publisher('/sim_init', EmptyMsg, latch=True)
            pub.publish(EmptyMsg())
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('move_robot')
    try:
        mover(rospy.get_param("~name"), rospy.get_param("~cellsize"))
    except rospy.ROSInterruptException:
        pass
