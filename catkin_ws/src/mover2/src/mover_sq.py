#!/usr/bin/env python

""" Example code of how to move a robot around the shape of a square, using tf. """

# we always import these
#import roslib; roslib.load_manifest('ex_move')
import rospy
import tf
import math

# recall: robots generally take base movement commands on a topic 
#  called "cmd_vel" using a message type "geometry_msgs/Twist"
from geometry_msgs.msg import Twist

class square:
    """ This example is in the form of a class. """

    def __init__(self):
        """ This is the constructor of our class. """
        # register this function to be called on shutdown
        rospy.on_shutdown(self.cleanup)

        self.listener = tf.TransformListener()

        # publish to cmd_vel
        self.p = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist)
        # give our node/publisher a bit of time to connect
        rospy.sleep(1.0)

        for i in range(4):
            # create a Twist message, fill it in to drive forward
            twist = Twist()
            twist.linear.x = 0.15;
            self.p.publish(twist)
            rospy.loginfo("moving forward")
            rospy.sleep(2)
            # stop robot            
            twist = Twist()
            self.p.publish(twist)
            # get angle to turn to
            th = (self.getAngle()+math.pi/2.0)
            if th > math.pi:
                th = (th-math.pi*2)
            # create a twist message, fill it in to turn
            twist = Twist()
            twist.angular.z = 1.57/2;
            self.p.publish(twist)
            rospy.loginfo("turning")
            # turn until we reach goal
            while( abs(self.getAngle()-th) > 0.1 ):
                rospy.sleep(0.05)

    def getAngle(self):
        try:
            ((x,y,z), rot) = self.listener.lookupTransform('/robot1/odom', 'robot1/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("AHHHH")
        (phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)
        print theta
        return theta
        
    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.p.publish(twist)

if __name__=="__main__":
    rospy.init_node('square')
    try:
        square()
    except:
        pass


