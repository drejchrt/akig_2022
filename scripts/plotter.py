#!/usr/bin/env python3
import math

from collections import deque

import rospy 
from geometry_msgs.msg import PoseWithCovarianceStamped as PWCS
from tf.transformations import euler_from_quaternion


from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse


MSG_TYPES = {
##    '/odometry/filtered': Odometry,
##    '/joint_states': JointState,
    '/amcl_pose': PWCS,
##    '/novatel_data/rawimudata_SIunits': Imu,
##    '/particlecloud': PoseArray
    }

##FIG, AX = plt.subplots()

deque_lim = 30 

STATES = {
    'pose_x': deque([],deque_lim),
    'pose_y': deque([],deque_lim),
    'pose_z': deque([],deque_lim),
    'ori_x': deque([],deque_lim),
    'ori_y': deque([],deque_lim),
    'ori_z': deque([],deque_lim),
    'ori_w': deque([],deque_lim),
    'cov': deque([],deque_lim)
    }

def error_ellipse(cov_mat):
    #    Covariance Matrix   
    #
    #        px py pz ox oy oz       
    #        -----------------
    # posx / 01 02 03 04 05 06 \
    # posy | 07 08 09 10 11 12 |
    # posz | 13 14 15 16 17 18 |
    # orix | 19 20 21 22 23 24 |
    # oriy | 25 26 27 28 29 30 |
    # oriz \ 31 32 33 34 35 36 /


    # TODO: Check, whether the calculation is correct
    qxx = cov_mat[0]
    qyy = cov_mat[7]
    qxy = cov_mat[1]
    mx = math.sqrt(qxx)
    my = math.sqrt(qyy)
    tau = 0.5 * math.atan2(2*qxy,qxx-qyy)
##    print(qxx,qyy,qxy)
##    print(tau*360/math.pi)
    q1 = (qxx + qyy)/2
    q2 = 0.5 * math.sqrt((qxx-qyy)**2+4*qxy**2)
    qmax = q1 + q2
    qmin = q1 - q2
    mmax = math.sqrt(qmax)
    mmin = math.sqrt(qmin)
    return mmax, mmin, tau



def plot_pose_with_covariance(msg):
    ax = plt.gca()
    
    # Update values in the deque
    STATES['pose_x'].append(msg.pose.pose.position.x)
    STATES['pose_y'].append(msg.pose.pose.position.y)
    STATES['pose_z'].append(msg.pose.pose.position.z)
    STATES['ori_x'].append(msg.pose.pose.orientation.x)
    STATES['ori_y'].append(msg.pose.pose.orientation.y)
    STATES['ori_z'].append(msg.pose.pose.orientation.z)
    STATES['ori_w'].append(msg.pose.pose.orientation.w)
    STATES['cov'].append(msg.pose.covariance)
    
    # plot/update track
    track, = plt.plot(
        STATES['pose_x'],
        STATES['pose_y'],
        color='tab:blue',
        label='Track',
        )

    # plot current pose and orientation (point)
    yaw = euler_from_quaternion([
            STATES['ori_x'][-1],
            STATES['ori_y'][-1],
            STATES['ori_z'][-1],
            STATES['ori_w'][-1]
        ])[2] * 360/math.pi
    print(yaw)
    pose, = plt.plot(
        STATES['pose_x'][-1],
        STATES['pose_y'][-1],
        marker=(3,0,yaw),
        markersize=20,
        color='tab:red',
        )
    
##    # plot ori (vector)
##    yaw = euler_from_quaternion([
##            STATES['ori_x'][-1],
##            STATES['ori_y'][-1],
##            STATES['ori_z'][-1],
##            STATES['ori_w'][-1]
##        ])[2]
##    dirx, diry = math.cos(yaw), math.sin(yaw)
##    ori = plt.quiver(
##        STATES['pose_x'][-1],
##        STATES['pose_y'][-1],
##        dirx,
##        diry,
##        units='inches'
##        )


    # Plot Error Ellipse
    mmax, mmin, tau = error_ellipse(STATES['cov'][-1])
    ellipse = Ellipse(
        xy=(STATES['pose_x'][-1],STATES['pose_y'][-1]),
        width=mmax,
        height=mmin,
        angle=tau*360/math.pi,
        fc='None',
        edgecolor='r'
        )
    
    ellipses = plt.gca().add_artist(ellipse)

    # Figure decorations
    plt.title("AKIG Plotter")
    plt.xlabel('[m]')
    plt.ylabel('[m]')
    
    # update figure
    plt.draw()
    plt.pause(0.0000000001)

    track.remove()
    pose.remove()
    ellipse.remove()
    

def listener():
    rospy.init_node('akig_plotter', anonymous=True)
    rospy.Subscriber('/amcl_pose', PWCS, plot_pose_with_covariance)
    rospy.spin()

if __name__=='__main__':
    plt.ion()
    plt.show()

    listener()
    
