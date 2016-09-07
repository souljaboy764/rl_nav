from numpy import *
import matplotlib.pyplot as plt
from pathplanner import *
import cPickle
import time



fig = plt.figure()
ax = plt.subplot(111)
#num = int((len(lines)/6.0))
#plt.plot([-4.5,-1.5,-1.5,-1.5,4.5,4.5,1.5,1.5,1.5,-4.5,-4.5],[-4.5,-4.5,1.5,-4.5,-4.5,4.5,4.5,-1.5,4.5,4.5,-4.5],'-k',label='Walls',lw=2.0)

#plt.axis('scaled')
#plt.axis([-5,5,-5,5])
ax.xaxis.set_visible(False)
ax.yaxis.set_visible(False)

[-3.6464170668541014, -4.027434003930228, 0.41088179795786112, 0], [-1.5, 3.0, 0.0, 10.0], [0.0, 0.0, -3.2815258508807728, 15.0]
#2.214980492175165, -3.376878827353146, -9.0546884567678756, 0], 3.05452561378479, -7.483584880828857, -0.10102158227646928, 14.0, 6.009502410888672, -1.8998010158538818, 14.666121751860494, 28.0
 
inp = [3.668602684166126, -6.816479171354969, 13.38859805243756,
		1.8782010078430176, -4.456195831298828, -3.3650417923055045,
	   0.0,0.0,0.0,0.0,0.0,0.0]
points = [ 4.034907817840576, -3.7889819145202637, -7.3185015696012536, 14]
to = 0.0
tf = 28.0
X = []
Y = []
coeffs = get_coeffs(inp,to,tf,points)
for t in  linspace(to,tf,num=200):
	point,kt = getPos(coeffs,to,tf,t,inp[1])
	X.append(point.z)
	Y.append(point.x)

plt.plot(X,Y,'0.7',linewidth=4)
plt.plot([inp[0],inp[3]], [inp[1],inp[4]], 'r.')
plt.plot([points[0]], [points[1]], 'b.')

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