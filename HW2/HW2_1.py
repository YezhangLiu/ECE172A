'''
ECE 172A, Homework 2 Robot Traversal
Author: regreer@ucsd.edu
For use by UCSD ECE 172A students only.
'''

import numpy as np
import matplotlib.pyplot as plt

initial_loc = np.array([0,0])
final_loc = np.array([100,100])
sigma = np.array([[60,0],[0,60]])
mu = np.array([[70, 40], [20, 50]])

def f(x, y):
	return ((final_loc[0]-x)**2 + (final_loc[1]-y)**2)/20000 + 10000*(1/(2*np.pi*np.linalg.det(sigma)))*np.exp(-.5*(np.matmul(np.array([x-mu[0,0], y-mu[0,1]]),np.matmul(np.linalg.pinv(sigma), np.atleast_2d(np.array([x-mu[0,0], y-mu[0,1]])).T)))[0]) + 10000*(1/(2*np.pi*np.linalg.det(sigma)))*np.exp(-.5*(np.matmul(np.array([x-mu[1,0], y-mu[1,1]]),np.matmul(np.linalg.pinv(sigma), np.array([x-mu[1,0], y-mu[1,1]])))))

x = np.linspace(0, 100, 100)
y = np.linspace(0, 100, 100)
z = f(x[:,None], y[None,:])
z = np.rot90(np.fliplr(z))
fig = plt.figure()
dy, dx = np.gradient(z)
plt.contour(x, y, z, 10)
plt.quiver(x, y, dx, dy)

# gradient descent to move the robot
step_size = 200
robot_loc = initial_loc
margin = 0.001

i = 0
j = 0
while i < 99 or j < 99: # and #abs(dx[i,j]) > margin and abs(dy[i,j]) > margin:
	
	i=int(robot_loc[0])
	j=int(robot_loc[1])
	# dynamic step size
	step_size = step_size + 10
	robot_loc[0]=robot_loc[0]-step_size*dy[i,j]
	robot_loc[1]=robot_loc[1]-step_size*dx[i,j]
	print(robot_loc)
	print(dx[i,j], dy[i,j])

	plt.plot(robot_loc[0], robot_loc[1], 'ro')

plt.show()

'''
ax = plt.axes(projection='3d')
ax.contour3D(x, y, z, 100)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_title('3D Contour')
plt.show()
'''
