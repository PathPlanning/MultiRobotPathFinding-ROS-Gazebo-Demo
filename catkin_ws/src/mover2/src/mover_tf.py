#!/usr/bin/env python

""" Example code of how to move a robot around the shape of a square, using tf. """
# we always import these
import roslib
import rospy
import tf
import math

# recall: robots generally take base movement commands on a topic 
#  called "cmd_vel" using a message type "geometry_msgs/Twist"
from geometry_msgs.msg import Twist

def square():
	""" This is the constructor of our class. """
	print 'inwut'
	listener = tf.TransformListener()
	print 'k1'

	# publish to cmd_vel
	p = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist)
	# give our node/publisher a bit of time to connect
	rospy.sleep(1.0)

	twist = Twist()

	#first = True
	for i in range(2):
		print 'hey'
		done = False
		first = 1
		print first

		while not done:
		    # create a Twist message, fill it in to drive forward
		    twist.linear.x = 0.2;
		    twist.angular.z = 0;
		    p.publish(twist)
		    rospy.loginfo("moving forward")

		    try:
				rospy.loginfo("moving forward1")				
				listener.waitForTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0), rospy.Duration(1.0))
				rospy.loginfo("moving forward2")				
				((new_x,y,z), rot) = listener.lookupTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0))
				rospy.loginfo("moving forward3")				

		    except (tf.LookupException, tf.ConnectivityException):
		        rospy.logerr("AHHHH")

		    #print 'mid!'


		    if first:
		    	orig_x=new_x
		    	first=0
		    	print 'first'       

		    done=bool(abs(new_x-orig_x)>0.8)

		    print done
		    print new_x-orig_x

		done = False
		first = 1

		while not done:

		    twist.angular.z = math.pi/4.0; #turns at pi/4 degrees/sec
		    twist.linear.x=0;
		    p.publish(twist)
		    rospy.loginfo("turning")

		    try:
		            ((x,y,z), rot) = listener.lookupTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0))


		    except (tf.LookupException, tf.ConnectivityException):
		            rospy.logerr("AHHHH")

		    (phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)

		    if first:
		        orig_ang=theta
		        first=0
		    done = bool(abs(theta-orig_ang)>=math.pi/2.0)   

		done = False
		first = 1
		print first

		while not done:
		    # create a Twist message, fill it in to drive forward
		    twist.linear.x = 0.2;
		    twist.angular.z = 0;
		    p.publish(twist)
		    rospy.loginfo("moving forward")

		    try:

		    	((x,new_y,z), rot) = listener.lookupTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0))

		    except (tf.LookupException, tf.ConnectivityException):
		        rospy.logerr("AHHHH")

		    #print 'mid!'


		    if first:
				orig_y=new_y
				first=0
				print 'first'       

		    done=bool(abs(new_y-orig_y)>0.8)

		    print done
		    print new_x-orig_x

		done = False
		first = 1

		while not done:

		    twist.angular.z = math.pi/16.0;
		    twist.linear.x=0;
		    p.publish(twist)
		    rospy.loginfo("turning")

		    try:
		            ((x,y,z), rot) = listener.lookupTransform('/robot1/odom', '/robot1/base_footprint', rospy.Time(0))


		    except (tf.LookupException, tf.ConnectivityException):
		            rospy.logerr("AHHHH")

		    (phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)

		    if first:
		        orig_ang=theta
		        first=0
		    done = bool(abs(theta-orig_ang)>=math.pi/2.0)      



if __name__=="__main__":
	rospy.init_node('square')
	try:
		square()
	except:
		pass
