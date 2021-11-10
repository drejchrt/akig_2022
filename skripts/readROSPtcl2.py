#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import tf2_ros
from sensor_msgs import point_cloud2
#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class PointsFromPointCloud:
    def __init__(self):
        self.cloud_in = PointCloud2()
        self.sub_ptcl = rospy.Subscriber("/scan_cloud", PointCloud2, self.callback, queue_size=10)

    def callback(self, msg):
        self.cloud_in = msg

    def reset(self):
        self.cloud_in.width = 0

    def process_pointcloud(self):
        
        while not rospy.is_shutdown():
            try:

                for point in point_cloud2.read_points(self.cloud_in):
                    # rospy.logwarn("x, y, z: %.1f, %.1f, %.1f" % (point[0], point[1], point[2]))
                    print("x, y, z: %.1f, %.1f, %.1f" % (point[0], point[1], point[2]))
                
                #self.pub_trans_ptcl.publish(cloud_out)
                self.reset()

            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.TransformException) as ex:

                rospy.logwarn("%s", ex)
                rospy.sleep(1)
                continue

        rospy.spin()


if __name__ == '__main__':

    try:
        rospy.init_node('processPointsFromPointCloud', anonymous=True)
        ptcl = PointsFromPointCloud()
        ptcl.process_pointcloud()

    except rospy.ROSInterruptException:
        pass
