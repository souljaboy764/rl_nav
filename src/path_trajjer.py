from numpy import *
import matplotlib.pyplot as plt
from pathplanner import *
import cPickle
import time



# f = open('/home/souljaboy764/Dropbox/CDC_RL/final_run/map1/run_8','r')
# lines = f.read().splitlines()
# berns = array([map(float,i.split()) for i in lines[::6]])
# mids = array([map(float,i.split()) for i in lines[1::6]])
# stops = array([map(float,i.split()) for i in lines[2::6]])
# q_state = array([map(float,i.split()) for i in lines[3::6]])
# states = array(lines[4::6])
# rl_action = array([map(float,i.split()) for i in lines[5::6]])

#plt.ion()
fig = plt.figure()
ax = plt.subplot(111)
#num = int((len(lines)/6.0))
plt.plot([0,0,8,8,4,4,4,0],[0,8,8,0,0,6,0,0],'-k',label='Walls',lw=2.0)
plt.axis('scaled')
plt.axis([-0.1,8.1,-0.1,8.1])
ax.xaxis.set_visible(False)
ax.yaxis.set_visible(False)


inp = [	2.0,2.0,tan(pi/4),
		6.0,2.0,-tan(pi/4),
	   0.0,0.0,0.0,0.0,0.0,0.0]
	
to = 2.0
tf = 28.0
X = []
Y = []
coeffs = get_coeffs_points(inp,to,tf,[[4.0,7.0,0.0,14.0]])
for t in  linspace(to,12.0,num=100):
	point,kt = getPos(coeffs,to,tf,t,2.0)
	X.append(point.z)
	Y.append(point.x)

plt.plot(X,Y,'k',linewidth=4)

# X=[]
# Y=[]
# for t in  linspace(12.0,14.0,num=100):
# 	point,kt = getPos(coeffs,to,tf,t,2.0)
# 	X.append(point.z)
# 	Y.append(point.x)

# plt.plot(X,Y,'r',linewidth=4)

angle = pi/20.0
num_angles = 3
inputs = []
orientation = arctan(kt)
x = X[-1]
y = Y[-1]
	
#for i in linspace(-num_angles*angle, num_angles*angle, 2*num_angles+1):
if True:
	i = num_angles*angle
			
	# inp = [0.0,0.0,0.0,cos(i), sin(i), tan(i),0.0,0.0,0.0,0.0,0.0,0.0]
	# to = 0.0
	# tf = 1.5
	# coeffs = get_coeffs_points(inp,to,tf,[])
	# X=[]
	# Y=[]
	# for t in  linspace(to,tf,num=100):
	# 	point,kt = getPos(coeffs,to,tf,t,0.0)
	# 	X.append(x+(point.z*cos(orientation) - point.x*sin(orientation)))
	# 	Y.append(y+(point.z*sin(orientation) + point.x*cos(orientation)))
	
	# plt.plot(X,Y,'g',linewidth=1)
	
	inp = [0.0,0.0,0.0,-cos(i), sin(i), -tan(i),0.0,0.0,0.0,0.0,0.0,0.0]
	to = 0.0
	tf = 1.5
	coeffs = get_coeffs_points(inp,to,tf,[])
	X=[]
	Y=[]
	for t in  linspace(to,tf,num=100):
		point,kt = getPos(coeffs,to,tf,t,0.0)
		X.append(x+(point.z*cos(orientation) - point.x*sin(orientation)))
		Y.append(y+(point.z*sin(orientation) + point.x*cos(orientation)))
	plt.plot(X,Y,'k',linewidth=4)



print Y[-1]
inp = [	X[-1],Y[-1],kt,
		6.0,2.0,-tan(pi/4),
	   0.0,0.0,0.0,0.0,0.0,0.0]
y=Y[-1]
to = 2.0
tf = 14.0
X = []
Y = []

coeffs = get_coeffs_points(inp,to,tf,[[4.0,7.0,0.0,4.0]])
for t in  linspace(to,12.0,num=100):
	point,kt = getPos(coeffs,to,tf,t,y)
	X.append(point.z)
	Y.append(point.x)

plt.plot(X,Y,'k',linewidth=4)
plt.show()	