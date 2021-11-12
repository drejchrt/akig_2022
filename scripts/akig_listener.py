#!/usr/bin/env python
import sys
import collections

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray

USAGE="""
Usage:

python topic_listener.py topic action

    topic - specifies which topic should the node listen to.
    available topics:
        /odometry/filtered
        /joint_states
        /amcl_pose
        /novatel_data/rawimudata_SIunits
        /particlecloud
        
"""
MSG_TYPES = {
    '/odometry/filtered': Odometry,
    '/joint_states': JointState,
    '/amcl_pose': PoseWithCovarianceStamped,
    '/novatel_data/rawimudata_SIunits': Imu,
    '/particlecloud': PoseArray
    }

class AkigMessage():
    def __init__(self, data):
        self.data = data
        
    def to_csv(self, sep=','):
        flat = flatten_dict(self.data)
        return sep.join([str(v) for _,v in flat.items()])

    def to_json(self):
        pass
    def to_numpy(self):
        pass
    def to_pandas(self):
        pass
    def to_pickle(self):
        pass
    
    def keys(self):
        return [k for k,_ in self.data.items()]
            
##    def flatten(self, sep='_'):
##        return AkigMessage(flatten_dict(self.data))
    

def flatten_dict(d, parent_key='', sep='_'):
    # Thanks Imran
    # https://stackoverflow.com/questions/6027558/flatten-nested-dictionaries-compressing-keys
    items = []
    for k, v in d.items():
        new_key = parent_key + sep + k if parent_key else k
        if isinstance(v, collections.MutableMapping):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


############################## ROS related methods #############################

def callback(data):
    if topic == '/odometry/filtered':
        akigmsg = AkigMessage({
                'header':{
                    'seq': data.header.seq,
                    'stamp': data.header.stamp,
                    'frame_id':data.header.frame_id
                    },
                'child_frame_id': data.child_frame_id,
                'pose': {
                    'pose':{
                        'position':{
                            'x': data.pose.pose.position.x,
                            'y': data.pose.pose.position.y,
                            'z': data.pose.pose.position.z,
                            },
                        'orientation':{
                            'x': data.pose.pose.orientation.x,
                            'y': data.pose.pose.orientation.y,
                            'z': data.pose.pose.orientation.z,
                            'w': data.pose.pose.orientation.w,
                            }
                        },
                    'covariance': data.pose.covariance
                    },
                'twist':{
                    'twist':{
                        'linear':{
                            'x':data.twist.twist.linear.x,
                            'y':data.twist.twist.linear.y,
                            'z':data.twist.twist.linear.z,
                            },
                        'angular':{
                            'x':data.twist.twist.angular.x,
                            'y':data.twist.twist.angular.y,
                            'z':data.twist.twist.angular.z,
                            }
                        },
                    'covarinace': data.twist.covariance
                    },
            })
        print(akigmsg.to_csv())
    elif topic == '/joint_states':
        pass
    elif topic == '/amcl_pose':
        pass
    elif topic == '/novatel_data/rawimudata_SIunits':
        pass
    elif topic == '/particlecloud':
        pass
    else:
        # TODO: Catch the exception and print usage
        raise NotImplementedError('Topic not implemented')


        
def listener():
    rospy.init_node('akig_listener', anonymous=True)
    rospy.Subscriber(topic, MSG_TYPES[topic], callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        topic = sys.argv[1]
    except IndexError:
        print('Error! Topic must be specified!')
        print(USAGE)
        exit(1)
    
    listener()
