#!/usr/bin/python
import roslib; roslib.load_manifest('rl_nav')
import rospy

from pathplanner import *
from numpy import *
from tf.transformations import *
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32MultiArray, Empty, String
from visualization_msgs.msg import Marker
from rl_nav.srv import ExpectedPath, ExpectedPathResponse
from gazebo_msgs.msg import ModelStates
import tf
import datetime
import time

class PLanner2D(object):

	def __init__(self):

		self.state = 0

		self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=100)
		self.status_pub = rospy.Publisher('/planner/status',String,queue_size=100)
		self.inp_pub = rospy.Publisher('/planner/global/path', Float32MultiArray,queue_size=100)
		self.safe_path_pub = rospy.Publisher("/rl/safe_action", Marker, queue_size=1);
		self.unsafe_path_pub = rospy.Publisher("/rl/unsafe_action", Marker, queue_size=1);
		self.pathSRV = rospy.Service('/planner/global/expected_path', ExpectedPath, self.sendLookahead)

		self.map = rospy.get_param('~map',-1)
		if(self.map==1):
			self.points = [[4.0,7.0,0.0,14.0],[6.0, 2.0,-tan(pi/4.0),28.0]] #map 1
		elif(self.map==2):
			self.points = [[4.0,4.0,tan(pi/4.0),14.0],[7.0,7.0,0.0,28.0]] #map 2
		else:
			self.points = []
		
		self.robotState = Pose()
		self.robotPTAMPose = Pose()
		self.robotPTAMWorldPose = Pose()

		self.rlSafePath = Marker()
		self.rlSafePath.id=0
		self.rlSafePath.lifetime=rospy.Duration(1)
		self.rlSafePath.header.frame_id = "/world"
		self.rlSafePath.header.stamp = rospy.Time.now()
		self.rlSafePath.ns = "pointcloud_publisher"
		self.rlSafePath.action = Marker.ADD
		self.rlSafePath.type = Marker.LINE_STRIP
		self.rlSafePath.color.g=1.0
		self.rlSafePath.color.a=1.0
		self.rlSafePath.scale.x=0.01
		
		self.rlUnsafePath = Marker()
		self.rlUnsafePath.id=0
		self.rlUnsafePath.lifetime=rospy.Duration(1)
		self.rlUnsafePath.header.frame_id = "/world"
		self.rlUnsafePath.header.stamp = rospy.Time.now()
		self.rlUnsafePath.ns = "pointcloud_publisher"
		self.rlUnsafePath.action = Marker.ADD
		self.rlUnsafePath.type = Marker.LINE_STRIP
		self.rlUnsafePath.color.r=1.0
		self.rlUnsafePath.color.a=1.0
		self.rlUnsafePath.scale.x=0.01
		

		self.ptam_pose_sub = rospy.Subscriber("/vslam/pose_world", PoseWithCovarianceStamped, self.receivePTAMPose)
		self.pose_sub = rospy.Subscriber("/my_pose", PoseStamped, self.receivePose)
		self.inp_sub = rospy.Subscriber('/planner/input', Float32MultiArray, self.receiveInput)
		self.inp_mul_sub = rospy.Subscriber('/planner/input/global', Empty, self.receiveInputGlobal)
		self.reset_sub = rospy.Subscriber('/planner/reset', Empty, self.receiveBreak)
		self.safe_traj_sub = rospy.Subscriber('/rl/safe_trajectories', Float32MultiArray, self.receiveSafeTrajs)
		self.unsafe_traj_sub = rospy.Subscriber('/rl/unsafe_trajectories', Float32MultiArray, self.receiveUnsafeTrajs)
		# self.PTAM_pose_sub = rospy.Subscriber("/vslam/pose", PoseWithCovarianceStamped, self.receivePTAMPose)
		gazeboModelStates_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.receiveGazeboModelStates)
		
		self.to = 0.0
		self.tf = 28.0

	def trajjer(self, data):
		traj_list=[]
		num_trajs = int(data[-1])
		size_trajs = int(len(data)/float(num_trajs))
		for i in range(num_trajs):
			inp = data[i*size_trajs:(i+1)*size_trajs]
			to = inp[-2]
			tf = inp[-1]
			coeffs = get_coeffs(inp[:-3],to,tf)
			
			t=to
			traj=[]
			while t<tf:
				point0,kt0 = getPos(coeffs,to,tf,t,0.0)
				traj.append(point0)
				t = t + 0.1
			traj_list = traj_list + traj
			traj.reverse()
			traj_list = traj_list + traj
		return traj_list

	def receiveUnsafeTrajs(self, trajectories):
		self.rlUnsafePath.pose = self.robotPTAMWorldPose
		self.rlUnsafePath.points = self.trajjer(trajectories.data)
		

	def receiveSafeTrajs(self, trajectories):
		self.rlSafePath.pose = self.robotPTAMWorldPose
		self.rlSafePath.points = self.trajjer(trajectories.data)
		

	def receivePTAMPose(self, pose):
		self.robotPTAMWorldPose = pose.pose.pose

		self.rlSafePath.header.stamp = rospy.Time.now()
		self.safe_path_pub.publish(self.rlSafePath)

		self.rlUnsafePath.header.stamp = rospy.Time.now()
		self.unsafe_path_pub.publish(self.rlUnsafePath)

		# br = tf.TransformBroadcaster()
		# br.sendTransform((pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z),
		# 				  [pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w],
		# 				  rospy.Time.now(),
		# 				  "camera_depth_optical_frame",
		# 				  "world")

	def receivePose(self, pose):
		self.robotPTAMPose = pose.pose

	def receiveGazeboModelStates(self,modelStates):
		self.robotState = modelStates.pose[-1]

	def receiveInput(self, inputArray):
		inp = inputArray.data
		
		status = String()
		status.data = "BUSY"
		self.status_pub.publish(status)

		self.state = 1
		to = inp[-2]
		tf = inp[-1]
		coeffs = get_coeffs(inp[:-3],to,tf)
		
		self.status_pub.publish(status)
		start_time = datetime.datetime.now()
		t=to
		r = rospy.Rate(10)
		cmd = Twist()
		while t<tf and self.state==1:
		
			dx,dy,dk = getVel(coeffs,to,tf,t)
			cmd.linear.x = inp[-3]*sqrt(dx**2 + dy**2)
			cmd.angular.z = inp[-3]*dk
			self.vel_pub.publish(cmd)
			r.sleep()
			t = t + 0.1
			self.status_pub.publish(status)
		time.sleep(0.3)
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())
		status.data = "DONE"
		self.status_pub.publish(status)

	#def commandRobot(self,event):

	def getQuadrant(self,angle):
		if 0<=angle<pi/2.0:
			return 1
		if pi/2.0<=angle<pi:
			return 1

	def receiveInputGlobal(self, empty):
		pose = [self.robotPTAMPose.position.x, self.robotPTAMPose.position.y, 
				tan(euler_from_quaternion([ self.robotPTAMPose.orientation.x,
										self.robotPTAMPose.orientation.y,
										self.robotPTAMPose.orientation.z,
										self.robotPTAMPose.orientation.w])[2]),0]

		if pose[0]>=4:
			points_done = 1
			self.points[1][-1]=14.0
		else:
			if pose[1]>=4:
				self.points[0][-1]=7.0
				self.points[1][-1]=21.0
			points_done = 0
		# 	if pose[1]<=4:
		# 		points_done = 3
		# 	else:
		# 		points_done = 2
		# else:
		# 	if pose[1]>=4:
		# 		points_done = 1
		# 	else:
		# 		points_done = 0
		points = [[self.robotPTAMPose.position.x, self.robotPTAMPose.position.y, 
				tan(euler_from_quaternion([ self.robotPTAMPose.orientation.x,
										self.robotPTAMPose.orientation.y,
										self.robotPTAMPose.orientation.z,
										self.robotPTAMPose.orientation.w])[2]),0]] + self.points[points_done:]
		
		status = String()
		self.state = 1
		inp = [points[0][0],points[0][1],points[0][2],
				points[-1][0],points[-1][1],points[-1][2],
			   0.0,0.0,0.0,0.0,0.0,0.0]
		# print inp
		# print points[1:-1]
		status.data = "BUSY"
		self.status_pub.publish(status)	
		
		to = self.to
		tf = points[-1][-1]
		if points[1:-1]==[]:
			coeffs = get_coeffs(inp,to,tf)
		else:
			coeffs = get_coeffs_points(inp,to,tf,points[1:-1])
		self.status_pub.publish(status)
		t=to

		self.status_pub.publish(status)
		self.state = 1
		r = rospy.Rate(10)
		cmd = Twist()
		while t<tf and self.state==1:
			
			dx,dy,dk = getVel(coeffs,to,tf,t)
			point1,kt1 = getPos(coeffs,to,tf,t,points[0][1])
			point2,kt2 = getPos(coeffs,to,tf,min(t+1,tf),points[0][1])
			message = Float32MultiArray()
			message.data= [ 0.0,0.0,0.0,
							point2.z - point1.z, point2.x - point1.x, tan(arctan(kt2) - arctan(kt1)),
				   			0.0,0.0,0.0,0.0,0.0,0.0,0.0]
			self.inp_pub.publish(message)
			#print self.robotState.position.x, self.robotState.position.y
			self.status_pub.publish(status)
			yaw = euler_from_quaternion([ self.robotPTAMPose.orientation.x,
										self.robotPTAMPose.orientation.y,
										self.robotPTAMPose.orientation.z,
										self.robotPTAMPose.orientation.w])[2]
			if fabs(yaw) < pi/2.0:
				cmd.linear.x = sign(dx) * sqrt(dx**2 + dy**2)
			elif fabs(yaw) > pi/2.0:
				cmd.linear.x = -sign(dx) * sqrt(dx**2 + dy**2)
			else:
				cmd.linear.x = sign(yaw)*sign(dy) * sqrt(dx**2 + dy**2)
			cmd.angular.z = dk
			
			self.vel_pub.publish(cmd)
			r.sleep()
			t = t + 0.1
			self.status_pub.publish(status)
				
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())
		status.data = "DONE1"
		self.status_pub.publish(status)

	def sendLookahead(self, req):
		pose = [self.robotPTAMPose.position.x, self.robotPTAMPose.position.y, 
				tan(euler_from_quaternion([ self.robotPTAMPose.orientation.x,
										self.robotPTAMPose.orientation.y,
										self.robotPTAMPose.orientation.z,
										self.robotPTAMPose.orientation.w])[2]),0]

		if pose[0]>4:
			points_done = 1
		else:
			points_done = 0
		points = [[self.robotPTAMPose.position.x, self.robotPTAMPose.position.y, 
				tan(euler_from_quaternion([ self.robotPTAMPose.orientation.x,
										self.robotPTAMPose.orientation.y,
										self.robotPTAMPose.orientation.z,
										self.robotPTAMPose.orientation.w])[2]),0]] + self.points[points_done:]
		inp = [points[0][0],points[0][1],points[0][2],
				points[-1][0],points[-1][1],points[-1][2],
			   0.0,0.0,0.0,0.0,0.0,0.0]
		
		to = self.to
		tf = points[-1][-1]
		coeffs = get_coeffs_points(inp,to,tf,points[1:-1])

		t=to

		
		point0,kt0 = getPos(coeffs,to,tf,t,points[0][1])
		point1,kt1 = getPos(coeffs,to,tf,t+1,points[0][1])
		point2,kt2 = getPos(coeffs,to,tf,t+2,points[0][1])
		message = Float32MultiArray()
		message.data = [ 0.0,0.0,0.0,
						point1.z - point0.z, point1.x - point0.x, tan(arctan(kt1) - arctan(kt0)),
						0.0,0.0,0.0,0.0,0.0,0.0] + \
						[ 0.0,0.0,0.0,
						point2.z - point0.z, point2.x - point0.x, tan(arctan(kt2) - arctan(kt0)),
						0.0,0.0,0.0,0.0,0.0,0.0] + \
						[ 0.0,0.0,0.0,
						point2.z - point1.z, point2.x - point1.x, tan(arctan(kt2) - arctan(kt1)),
						0.0,0.0,0.0,0.0,0.0,0.0]
		return ExpectedPathResponse(message)
				

	def receiveBreak(self, empty):
		self.state = 0
		self.vel_pub.publish(Twist())
		self.vel_pub.publish(Twist())

rospy.init_node('PlannerNode')
PLanner2D()
rospy.spin()
