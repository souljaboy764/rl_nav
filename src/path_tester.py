from numpy import *
import matplotlib.pyplot as plt
from pathplanner import *
import cPickle
import time



fig = plt.figure()
ax = plt.subplot(111)
#num = int((len(lines)/6.0))
plt.plot([-4.5,-1.5,-1.5,-1.5,4.5,4.5,1.5,1.5,1.5,-4.5,-4.5],[-4.5,-4.5,3.0,-4.5,-4.5,4.5,4.5,-3.0,4.5,4.5,-4.5],'-k',label='Walls',lw=2.0)

plt.axis('scaled')
plt.axis([-5,5,-5,5])
ax.xaxis.set_visible(False)
ax.yaxis.set_visible(False)

inp = [	-3.5,3.5,tan(1.27409035),
		3.5,3.5,tan(1.27409035),
	   0.0,0.0,0.0,0.0,0.0,0.0]
to = 0.0
tf = 30.0
X = []
Y = []
coeffs = get_coeffs_points(inp,to,tf,[[-1.5,5.0,0.0,10.0],[1.5,-5.0,0.0,20.0]])
for t in  linspace(to,tf,num=200):
	point,kt = getPos(coeffs,to,tf,t,inp[1])
	X.append(point.z)
	Y.append(point.x)

plt.plot(X,Y,'0.7',linewidth=4)

# inp = [ 0,0,-tan(1.27409035),
# 		3.5,3.5,tan(1.27409035),
# 	    0.0,0.0,0.0,0.0,0.0,0.0]
# to = 0.0
# tf = 15.0
# X = []
# Y = []
# coeffs = get_coeffs_points(inp,to,tf,[[1.5,-4.0,0.0,7.0]])
# for t in  linspace(to,tf,num=200):
# 	point,kt = getPos(coeffs,to,tf,t,inp[1])
# 	X.append(point.z)
# 	Y.append(point.x)

# plt.plot(X,Y,'0.7',linewidth=4)


plt.show()	