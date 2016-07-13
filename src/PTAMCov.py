#!/usr/bin/python
import roslib; roslib.load_manifest('turtlebot_nav')
import rospy

from numpy import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import *

class ErrorNode(object):

	def __init__(self):

		#pose_sub = rospy.Subscriber("/vslam/pose", PoseWithCovarianceStamped, self.receivePose)
		wpose_sub = rospy.Subscriber("/vslam/pose_world", PoseWithCovarianceStamped, self.receivePoseWorld)

	def receivePose(self, pose):
		#print linalg.norm(pose.pose.covariance)
		#print trace(reshape(pose.pose.covariance,(6,6)))
		angles = euler_from_quaternion([pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w])
		print "P:",pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z

	def receivePoseWorld(self, pose):
		#print linalg.norm(pose.pose.covariance)
		#print trace(reshape(pose.pose.covariance,(6,6)))
		angles = euler_from_quaternion([pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w])
		print "W:",pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z

rospy.init_node('ErrorNode')
ErrorNode()
rospy.spin()
