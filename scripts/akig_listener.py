#!/usr/bin/env python3
import sys
import collections

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray

USAGE = """
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
    """
    This class stores a message in a dictionary and provides extra functionality
    to the data, so that one can easily convert the data to various formats.

    #TODO : check for pose arrays...
    """
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
    """
    This function removes nested dictionaries from specified dictionary and
    puts them in the 'top' level. The keys of the nested dictionaries are
    are prefixed by their parent key.

    Example:
    {'foo':'bar','baz':{'a':1,'b':8}} => {'foo':'bar','baz.a':1,'baz.b':8}

    param d: dictionary to be flattened
    param parent_key: key, that contains nested dict
    param sep: separator that is used in in keys between parents and children
    
    Thanks Imran
    https://stackoverflow.com/questions/6027558/flatten-nested-dictionaries-compressing-keys
    """
    items = [] # list containing the keys
    for k, v in d.items(): # iterate over all items
        new_key = parent_key + sep + k if parent_key else k # get new key name
        # check if current value is a nested dict
        if isinstance(v, collections.MutableMapping):
            # if dict => rescursive call
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            # else append the new_key to the items
            items.append((new_key, v))
    return dict(items) # convert to dict


############################## ROS related methods #############################

def callback(data):
    """
    This function specifies what happens when the subscriber node captures a message
    on the topic it's listening to. In this case the function converts the message
    to an AkigMessage object that provides addtional functionality (such as convert
    to CSV etc.) to the data by re-structuring the data.

    param data: captured Message
    """
    
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
        
    elif topic == '/joint_states':
        akigmsg = AkigMessage({
            'header':{
                    'seq': data.header.seq,
                    'stamp': data.header.stamp,
                    'frame_id':data.header.frame_id
                    },
            'name':data.name,
            'position':data.position,
            'velocity':data.velocity,
            'effort':data.effort
            })
    elif topic == '/amcl_pose':
        akigmsg = AkigMessage({
            'header':{
                'seq': data.header.seq,
                'stamp': data.header.stamp,
                'frame_id':data.header.frame_id
                    },
            'pose':{
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
                'covarinace':data.pose.covariance
                },
            })
    elif topic == '/novatel_data/rawimudata_SIunits':
        akigmsg = AkigMessage({
            'header':{
                'seq': data.header.seq,
                'stamp': data.header.stamp,
                'frame_id':data.header.frame_id
                },
            'orientation':{
                'x': data.orientation.x,
                'y': data.orientation.y,
                'z': data.orientation.z,
                'w': data.orientation.w,
                },
            'orientation_covariance': data.orientation_covariance,
            'angular_velocity': {
                'x': data.angular_velocity.x,
                'y': data.angular_velocity.y,
                'z': data.angular_velocity.z,
                },
            'angular_velocity_covariance': data.angular_velocity_covariance,
            'linear_acceleration': {
                'x': data.linear_acceleration.x,
                'y': data.linear_acceleration.y,
                'z': data.linear_acceleration.z,
                },
            'linear_acceleration_covariance': data.linear_acceleration_covariance
            
            })
    elif topic == '/particlecloud':
        akigmsg = AkigMessage({
            'header':{
                'seq': data.header.seq,
                'stamp': data.header.stamp,
                'frame_id':data.header.frame_id
                },
            'poses': [{
                'position':{
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z,
                    },
                'orientation':{
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w,
                }} for pose in data.poses]
            })
    else:
        # TODO: Catch the exception and print usage
        raise NotImplementedError('Topic not implemented')

    # Work with the akig message object

    print(akigmsg.to_csv())

    # e.g. write to a file or plot the data
    
    
        
def listener():
    # create the node
    rospy.init_node('akig_listener', anonymous=True)
    # Subscribe the node to specified topic
    rospy.Subscriber(topic, MSG_TYPES[topic], callback)
    # Loopy loop
    rospy.spin()

if __name__ == '__main__':
    try:
        topic = sys.argv[1]
    except IndexError:
        print('Error! Topic must be specified!')
        print(USAGE)
        exit(1)
    
    listener()
