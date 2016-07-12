#!/usr/bin/python
import roslib; roslib.load_manifest('turtlebot_nav')
import rospy

from numpy import *
from geometry_msgs.msg import PoseWithCovarianceStamped

class ErrorNode(object):

	def __init__(self):

		pose_sub = rospy.Subscriber("/vslam/pose_world", PoseWithCovarianceStamped, self.receivePose)

	def receivePose(self, pose):
		#print linalg.norm(pose.pose.covariance)
		print trace(reshape(pose.pose.covariance,(6,6)))

rospy.init_node('ErrorNode')
ErrorNode()
rospy.spin()
