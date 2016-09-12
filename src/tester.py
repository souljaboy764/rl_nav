from numpy import *
import matplotlib.pyplot as plt
from pathplanner import *
import cPickle
import time




angle = pi/90.0
num_angles = 14
inputs=[]
orientation = 0

for i in linspace(0, num_angles*angle, num = num_angles+1):
	inp = [0.0,0.0,0.0, cos(i), sin(i), tan(i),0.0,0.0,0.0,0.0,0.0,0.0]
	to = 0.0
	tf = 1.5
	coeffs = get_coeffs(inp,to,tf)
	t = 0
	X = []
	Y = []
	while t<tf:
		point,k = getPos(coeffs,to,tf,t,inp[1])
		X.append(point.z)
		Y.append(point.x)
		t = t + 0.1
	plt.plot(X,Y,'r-')
	inp[3] = -inp[3]
	inp[5] *= -inp[5]
	coeffs = get_coeffs(inp,to,tf)
	t = 0
	X = []
	Y = []
	while t<tf:
		point,k = getPos(coeffs,to,tf,t,inp[1])
		X.append(point.z)
		Y.append(point.x)
		t = t + 0.1
	plt.plot(X,Y,'b-')

plt.show()
