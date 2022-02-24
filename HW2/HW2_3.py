'''
ECE 172A, Homework 2 Robot Kinematics
Author: regreer@ucsd.edu
For use by UCSD ECE 172A students only.
'''

from tkinter import E
import numpy as np
import matplotlib.pyplot as plt

def forwardKinematics(theta0, theta1, theta2, l0, l1, l2):
    x_1 = np.cos(theta0) * l0
    y_1 = np.sin(theta0) * l0
    x_2 = np.cos(theta1) * l1 + x_1
    y_2 = np.sin(theta1) * l1 + y_1
    x_e = np.sin(theta2) * l2 + x_2
    y_e = np.cos(theta2) * l2 + y_2
    return x_1,y_1,x_2,y_2,x_e,y_e

def inverseKinematics(l0,l1,l2,x_e_target,y_e_target):
    '''
    This function is supposed to implement inverse kinematics for a robot arm
    with 3 links constrained to move in 2-D. The comments will walk you through
    the algorithm for the Jacobian Method for inverse kinematics.

    INPUTS:
    l0, l1, l2: lengths of the robot links
    x_e_target, y_e_target: Desired final position of the end effector 

    OUTPUTS:
    theta0_target, theta1_target, theta2_target: Joint angles of the robot that
    take the end effector to [x_e_target,y_e_target]
    '''

    # Initialize for the plots:
    end_effector_positions = []

    # Initialize the thetas to some value
    #theta0 = np.pi/6
    theta0 = np.pi/3
    theta1 = 0
    theta2 = 0
    # Obtain end effector position x_e, y_e for current thetas: 
    # HINT: use your ForwardKinematics function   
    x_1,y_1,x_2,y_2,x_e,y_e = forwardKinematics(theta0, theta1, theta2, l0, l1, l2)
    
    while np.sqrt((x_e - x_e_target)**2 + (y_e - y_e_target)**2) > 0.01: # Replace the '1' with a condition that checks if your estimated [x_e,y_e] is close to [x_e_target,y_e_target]
        
        # Calculate the Jacobian matrix for current values of theta
        # HINT: write a function for doing this      
        Jmat = Jacob(theta0,theta1,theta2,l0,l1,l2)
        
        # Calculate the pseudo-inverse of the jacobian (HINT: numpy pinv())     
        JamtInv =np.linalg.pinv(Jmat)

        # Update the values of the thetas by a small step
        diff = np.array([x_e_target - x_e, y_e_target - y_e])
        loss = np.dot(JamtInv, diff)
        theta0 = theta0 + 0.1 * loss[0]
        theta1 = theta1 + 0.1 * loss[1]
        theta2 = theta2 + 0.1 * loss[2]

        # Obtain end effector position x_e, y_e for the updated thetas:
        x_1,y_1,x_2,y_2,x_e,y_e = forwardKinematics(theta0, theta1, theta2, l0, l1, l2)
        
        # If you would like to visualize the iterations, draw the robot using drawRobot. 
        #drawRobot2(x_1,y_1,x_2,y_2,x_e,y_e)
        
        # Save end effector positions for the plot:
        end_effector_positions.append([x_e, y_e])

    # Plot the final robot pose
    x_1,y_1,x_2,y_2,x_e,y_e = forwardKinematics(theta0, theta1, theta2, l0, l1, l2)
    drawRobot(x_1,y_1,x_2,y_2,x_e,y_e)
    
    # Plot the end effector position through the iterations
    plotx = []
    ploty = []
    for i in end_effector_positions:
        plotx.append(i[0])
        ploty.append(i[1])

    plt.plot(plotx,ploty,color='r')
    plt.savefig("ahaha.png")
    return theta0, theta1, theta2
    
def Jacob(theta0, theta1, theta2, l0, l1, l2):
    #x_e = np.sin(theta2) * l2 + np.cos(theta1) * l1 + np.cos(theta0) * l0
    #y_e = np.cos(theta2) * l2 + np.sin(theta1) * l1 + np.sin(theta0) * l0
    Jmatrix = np.array([[-np.sin(theta0) * l0, -np.sin(theta1) * l1, np.cos(theta2) * l2], \
                [np.cos(theta0)*l0, np.cos(theta1)*l1, -np.sin(theta2)*l2]])
    return Jmatrix

def drawRobot2(x_1,y_1,x_2,y_2,x_e,y_e):
    x_0, y_0 = 0, 0
    plt.plot([x_0, x_1, x_2, x_e], [y_0, y_1, y_2, y_e], lw=4.5)
    plt.scatter([x_0, x_1, x_2, x_e], [y_0, y_1, y_2, y_e], color='r')
    plt.show(block=False)
    plt.pause(.05)

def drawRobot(x_1,y_1,x_2,y_2,x_e,y_e):
    x_0, y_0 = 0, 0
    plt.plot([x_0, x_1, x_2, x_e], [y_0, y_1, y_2, y_e], lw=4.5)
    plt.scatter([x_0, x_1, x_2, x_e], [y_0, y_1, y_2, y_e], color='r')
    plt.show()

# 3.1
#x_1,y_1,x_2,y_2,x_e,y_e = forwardKinematics(np.pi/3, np.pi/12, -np.pi/6, 3, 5, 7)
#x_1,y_1,x_2,y_2,x_e,y_e = forwardKinematics(np.pi/4, np.pi/4, np.pi/4, 3, 5, 2)
#drawRobot(x_1,y_1,x_2,y_2,x_e,y_e)
# 3.2
t0,t1,t2 = inverseKinematics(10,10,10,6,12)
print(t0,t1,t2)
