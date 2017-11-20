#!/usr/bin/env python

""" Example code of how to move a robot around the shape of a square, using tf. """

# we always import these
#import roslib; roslib.load_manifest('ex_move')
import rospy
import tf
import math
import numpy as np
import sys

# recall: robots generally take base movement commands on a topic 
#  called "cmd_vel" using a message type "geometry_msgs/Twist"
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

current_pose = Pose2D()

class square:
    """ This example is in the form of a class. """

    def callbackOdom(self,msg):
        # linear position
        current_pose.x = msg.pose.pose.position.x;
        current_pose.y = msg.pose.pose.position.y;
        # quaternion to RPY conversion
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w);
        euler = tf.transformations.euler_from_quaternion(q)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        
        # angular position
        current_pose.theta = yaw;


    def driveForwardOdom(self,distance):
        #wait for the listener to get the first message     
        self.listener = tf.TransformListener()        
        self.listener.waitForTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0), rospy.Duration(1.0))

        try:
            (start_trans,start_rot) = self.listener.lookupTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        #we will be sending commands of type "twist"
        base_cmd=Twist()
        #the command will be to go forward at 0.25 m/s
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = 0.1;

        done = False;
        while done==False:
          #send the drive command
          self.p.publish(base_cmd)

          try:
              (cur_trans,cur_rot) = self.listener.lookupTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0))
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
              pass
          #see how far we've traveled
          
          a=np.asarray(start_trans)      
          b=np.asarray(cur_trans) 

          dist_moved = np.linalg.norm(a-b)
          rospy.loginfo(dist_moved)
          if(dist_moved > distance): done = True

    def turnOdom(self, radians):
        
        while(radians < 0): 
            radians =radians+ 2*math.pi;
        while(radians > 2*math.pi): 
            radians =radians- 2*math.pi;

 

        #wait for the listener to get the first message     
        self.listener = tf.TransformListener()        
        self.listener.waitForTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0), rospy.Duration(1.0))

        try:
            (start_trans,start_rot) = self.listener.lookupTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(start_rot)
            start_yaw = euler[2]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRROR")  
            pass
      
        #we will be sending commands of type "twist"
        base_cmd=Twist()
        #command will be to turn at 0.75 rad/s
        base_cmd.linear.x = base_cmd.linear.y = 0.0;
        base_cmd.angular.z = 0.1;

         
        done = False;
        while done==False:
          #send the drive command
          self.p.publish(base_cmd)

          try:
              (cur_trans,cur_rot) = self.listener.lookupTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0))
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRROR")  
            pass

          #see how far we've traveled
          euler = tf.transformations.euler_from_quaternion(cur_rot)
          cur_yaw = euler[2]
          # rospy.loginfo("start_yaw: ")
          # rospy.loginfo(start_yaw)
          # rospy.loginfo("cur_yaw: ")
          # rospy.loginfo(cur_yaw)

          angle_turned=cur_yaw-start_yaw

          rospy.loginfo(angle_turned)
          #if (math.fabs(angle_turned) < 1.0e-2):
          #   continue

          if (angle_turned > math.pi):
              angle_turned -= 2 * math.pi;
          if (angle_turned <= -math.pi):
              angle_turned += 2 * math.pi;
#
          rospy.loginfo("angle_turned2 "+str(angle_turned))

          if (angle_turned > radians): done = True;

    

    def __init__(self,robot):
        self.robot = robot
        """ This is the constructor of our class. """
        # register this function to be called on shutdown
        #rospy.on_shutdown(self.cleanup)

        #odom_sub = rospy.Subscriber('/robot1/odom', Odometry, self.callbackOdom)
        rospy.loginfo("do")
        # publish to cmd_vel
        self.p = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist)
        # give our node/publisher a bit of time to connect
        rospy.sleep(1.0)

        rospy.loginfo("turn 90")
        self.turnOdom(math.pi / 2);

        rospy.loginfo("turn 45")
        self.turnOdom(math.pi / 4);

        rospy.loginfo("move forward")
        self.driveForwardOdom(1.5);

        rospy.loginfo("turn 90")
        self.turnOdom(math.pi / 2);

        rospy.loginfo("move forward")
        self.driveForwardOdom(1.5);

        rospy.loginfo("turn 90")
        self.turnOdom(math.pi/2);
        rospy.loginfo("move forward")
        self.driveForwardOdom(1.5);

        rospy.loginfo("turn 90")
        self.turnOdom(math.pi/2);
        rospy.loginfo("move forward")
        self.driveForwardOdom(1.5);


        rospy.loginfo("turn 90")
        self.turnOdom(math.pi/2);
        rospy.loginfo("move forward")
        self.driveForwardOdom(1.5);
        
        rospy.loginfo("turn 45")
        self.turnOdom(math.pi/4);
        rospy.loginfo("move forward")
        self.driveForwardOdom(1.5);

        rospy.loginfo("turn 45")
        self.turnOdom(math.pi / 4);
        rospy.loginfo("move forward")
        self.driveForwardOdom(1.5);

        rospy.loginfo("turn 45")
        self.turnOdom(math.pi / 4);
        rospy.loginfo("move forward")
        self.driveForwardOdom(1.5);

        rospy.loginfo("turn 45")
        self.turnOdom(math.pi / 4);
        rospy.loginfo("move forward")
        self.driveForwardOdom(1.5);

        
    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.p.publish(twist)

if __name__=="__main__":
    rospy.init_node('square')
    rospy.Rate(10)
    try:
        square(sys.argv[1])
    except:
        pass


