#!/usr/bin/python
import roslib; roslib.load_manifest('turtlebot_nav')
import rospy
import cPickle
from numpy import *
from turtlebot_nav.srv import SLpredictionResponse, SLprediction
from std_msgs.msg import Bool

class SLPredictor(object):

	def __init__(self):
		self.predictionSRV = rospy.Service('/rl/sl_prediction', SLprediction, self.sendSLPrediction)
		self.svm = cPickle.load(open('best_SVM.pkl','rb'))
	
	def sendSLPrediction(self, req):
		return SLpredictionResponse(Bool(bool(self.svm.predict([req.stateAction.data])[0])))

rospy.init_node('SLPredictorNode')
SLPredictor()
rospy.spin()
