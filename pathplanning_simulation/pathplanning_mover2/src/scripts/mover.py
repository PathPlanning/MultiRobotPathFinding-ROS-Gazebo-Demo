#!/usr/bin/env python
# coding=utf-8
import roslib
import rospy
import math
import tf
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from gazebo_msgs.msg import ModelState
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
        yaw = -yaw
        quaternion = tf.transformations.quaternion_from_euler(0, 0, -yaw)
        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]

    """Для отладки раскомментировать строки 37, 52, 53"""

    def move_forward(self, array_of_coords, time):
        if (len(array_of_coords) == 0):
            return
        # extime = timeit.default_timer()
        for element in array_of_coords:
            exectime = timeit.default_timer()  # для уменьшения погрешности во времени выполнения
            print element
            self.pose.position.x = element[0]
            self.pose.position.y = element[1]
            self.state.model_name = "robot_" + self.robot
            self.state.pose = self.pose
            self.pub.publish(self.state)
            # для организации равномерного движения заставляем rospy спать каждую итерацию
            # время сна = время на команду / количество промежуточных точек
            # для более точного рассчета вычитаем из времени сна, выделенного на одну итерацию цикла
            # время, которое заняли команды этой итерации.
            # Отправка в топик занимает большую часть времени
            rospy.sleep(float(time / len(array_of_coords)) - (timeit.default_timer() - exectime))
            # print("Executed in: " + str(timeit.default_timer() - extime))
            # rospy.sleep(5)

    def calculate_coords(self, command):
        # находим k и x для формулы линейного уравнения y = kx +b
        points = [(command[2], command[3]), (command[4], command[5])]
        x_coords, y_coords = zip(*points)
        A = vstack([x_coords, ones(len(x_coords))]).T
        m, c = lstsq(A, y_coords)[0]
        delta = 0.02  # тут можно редактировать
        # если стартовые позиции численно больше конечных - нам нужно двигаться влево или вниз
        if command[2] > command[4]:
            deltax = -delta
        elif command[2] < command[4]:
            deltax = delta
        # delta нельзя присвоить 0, тк итератор должен быть ненулевым числом
        else:
            deltax = delta
        if command[3] > command[5]:
            deltay = -delta
        elif command[3] < command[5]:
            deltay = delta
        else:
            deltay = delta
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
                    v1 = np.array([int(row[2]) - int(row[0]),
                                   int(row[3]) - int(row[1])])
                    # negative x axis
                    v2 = np.array([-1, 0])
                    alpha = self.py_ang(v1, v2)  # угол
                    vector_length = la.norm(v2)  # длина вектора передвижения
                    commands.append([alpha, vector_length,
                                     int(row[0]) * self.cellsize + self.cellsize / 2,
                                     int(row[1]) * self.cellsize + self.cellsize / 2,
                                     int(row[2]) * self.cellsize + self.cellsize / 2,
                                     int(row[3]) * self.cellsize + self.cellsize / 2,
                                     float(row[4])])
        except Exception as e:
            print e.message
        return commands

    def __init__(self, robot, cellsize):

        self.pub = rospy.Publisher("/gazebo/set_model_state", ModelState, latch=True)
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
