#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rospy.init_node('image_tracking_control')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
bridge = CvBridge()

last_side = 1;

# Returns the coordinates of the centroid of a color mass in an image,
# after having applied a specified filter. Returns a tuple of:
# (x coordinate, y coordinate, had an error?)
def find_mass(cap):
	M = cv2.moments(cap)
	if M["m00"] != 0:
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		return (cX, cY, False)
	else: 
		return (-1, -1, True)


def callback(data):
	global pub
	global last_side

	# do stuff with the data
	cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    
	#interpret image
	grayscaled = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	chopped = grayscaled[-100::]
	_, mask = cv2.threshold(chopped, 80, 255, cv2.THRESH_BINARY_INV)
	cX, _, err = find_mass(mask)
    
	move = Twist()
	# pid control, aiming for cX centered
	if err:
		# We can't see the road
		move.linear.x = 0
		move.angular.z = 1 * last_side
	else:
		# PID
		width = data.width
		x_err = (width / 2 - cX)
		move.linear.x = 2
		move.angular.z = x_err / 40


	#publish
	pub.publish(move)

sub = rospy.Subscriber('/rrbot/camera1/image_raw', Image, callback)
    
# Don't close the interpreter
rospy.spin()
