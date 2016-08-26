#!/usr/bin/python
import roslib; roslib.load_manifest('rl_nav')
import rospy
from numpy import *
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_msgs.msg import Empty
import os

class LocationPrinterNode(object):

	def __init__(self):
		gazeboModelStates_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.receiveGazeboModelStates)
		ptamPose_sub = rospy.Subscriber("/vslam/pose_world", PoseWithCovarianceStamped, self.receivePTAMPose)
		start_sub = rospy.Subscriber("/printer/start", Empty, self.receiveStart)
		self.pose = Pose()
		self.robotState = Pose()
		self.start = False
	
	def receiveStart(self, empty):
		self.start = True

	def receivePTAMPose(self,pose):
		self.pose = pose.pose.pose

	def receiveGazeboModelStates(self,modelStates):
		self.robotState = modelStates.pose[-1]

	def spin(self):
		filename = os.path.expanduser('~') + '/path'
		try:
			os.remove(filename)
		except OSError:
			pass
		f = open(filename,'w')
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.start:
				print -self.pose.position.z, -self.pose.position.x, self.robotState.position.x, self.robotState.position.y
				f.write(' '.join(map(str,[-self.pose.position.z, -self.pose.position.x, self.robotState.position.x, self.robotState.position.y])))
				f.write('\n')
			r.sleep()

rospy.init_node('LocationPrinterNode')
LocationPrinterNode().spin()
