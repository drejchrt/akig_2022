#!/usr/bin/env python3
import sys
import math

from collections import deque

import rospy 
from geometry_msgs.msg import PoseWithCovarianceStamped as PWCS
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse



# Dictionary, that translates topic to message type classes
MSG_TYPES = {
    '/odometry/filtered': Odometry,
##    '/joint_states': JointState,
    '/amcl_pose': PWCS,
##    '/novatel_data/rawimudata_SIunits': Imu,
##    '/particlecloud': PoseArray
    }



def error_ellipse(cov_mat):
    """
    Takes 36 float values representing a 6x6 covariance matrix that contains
    variances of a position and a orientation and outputs the parameters of
    an error ellipse:
    1) Semi major axis (mmax)
    2) Semi minor axis (mmin)
    3) Orientation (tau)
    """
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
    """
    Callback function for the subscriber node.
    It pushes the values from the message into the STATES variable
    and plots the values from current message

    """
    # Figure decorations
    plt.title(f'AKIG Plotter - {topic}')
    plt.xlabel('[m]')
    plt.ylabel('[m]')
    
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
    if plot_track:
        track, = plt.plot(
            STATES['pose_x'],
            STATES['pose_y'],
            color='tab:blue',
            label='Track',
            )

    # plot current pose and orientation (point)
    if plot_pose:
        yaw = euler_from_quaternion([
                STATES['ori_x'][-1],
                STATES['ori_y'][-1],
                STATES['ori_z'][-1],
                STATES['ori_w'][-1]
            ])[2] * 360/math.pi
        
        pose, = plt.plot(
            STATES['pose_x'][-1],
            STATES['pose_y'][-1],
            marker=(3,0,yaw),
            markersize=20,
            color='tab:red',
            label='pose and orientation'
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
    if plot_ellipse:
        mmax, mmin, tau = error_ellipse(STATES['cov'][-1])
        ellipse = Ellipse(
            xy=(STATES['pose_x'][-1],STATES['pose_y'][-1]),
            width=mmax,
            height=mmin,
            angle=tau*360/math.pi,
            fc='None',
            edgecolor='r',
            label='Error Ellipse'
            )
        
        ellipses = plt.gca().add_artist(ellipse)


    # aggregate handles for legend
    legend = [
        track,
        pose,
        ellipse
        ]

    # update figure
    ax.legend(handles=legend, loc='lower right')
    plt.draw()
    plt.pause(0.00001)

    if plot_track:
        track.remove()
    if plot_pose:
        pose.remove()
    if plot_ellipse and False:
        ellipse.remove()
    

def listener():
    rospy.init_node('akig_plotter', anonymous=True)
    rospy.Subscriber(topic, MSG_TYPES[topic], plot_pose_with_covariance)
    rospy.spin()

################################################################################

if __name__=='__main__':   
    # handle arguments
    # store the user's decisions, what should be plotted 
    plot_pose = rospy.get_param('/akig_plotter/plot_pose', default=True)
    plot_track = rospy.get_param('/akig_plotter/plot_track', default=True)
##    plot_ori = rospy.get_param('/akig_plotter/plot_ori')
    plot_ellipse = rospy.get_param('/akig_plotter/plot_cov', default=True)
    # topic from which the pose is taken (currently /amcl_pose and /odometry/filtered)
    topic = rospy.get_param('/akig_plotter/topic', default='/amcl_pose')
    # How many states should be stored 
    deque_lim = rospy.get_param('/akig_plotter/deque_limit', default=500)

    # This dictionary stores a state. Deque = Double Ended Queue.
    # Deque resembles shift register
    STATES = {
        'pose_x': deque([],deque_lim),
        'pose_y': deque([],deque_lim),
        'pose_z': deque([],deque_lim),
        'ori_x': deque([],deque_lim),
        'ori_y': deque([],deque_lim),
        'ori_z': deque([],deque_lim),
        'ori_w': deque([],deque_lim),
        'cov':deque([],deque_lim)
        }
    
    plt.ion()   # Turn on interactive mode, so that the plot remains opened
    plt.show()  # Open the (for now) empty figure

    listener()  # Run the node
    
