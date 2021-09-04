#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid 
import struct
import numpy as np
# import rosbag
# path = "/home/kseniia/downloads/semantic_pseudo_point_cloud.bag"
# bag = rosbag.Bag(path)

# topic = "/stereo_depth/point_cloud"
# for topic, msg, t in bag.read_messages(topics = topic):
#     data = unpack_points(msg)
#     cmap  = points_to_costmap(msg, classes, costs)

# bag.close()

class cloud_to_costmap():

    # fence - изгородь, pole - столб, vegetation - растительность
    # terrain - местность
    keys = [         'road',     'sidewalk',   'building',   'wall', 'fence',       'pole',
            'traffic light', 'traffic sign', 'vegetation','terrain',   'sky',     'person',
                    'rider',          'car',      'truck',    'bus', 'train', 'motorcycle', 
                'bicycle']
    costs = [50,   0, 100, 100, 100, 100,
            50, 100,  75,  75,   0, 100,
            100, 100, 100, 100, 100, 100,
            100]              
    classes = {k:i for k, i in zip(keys, range(len(keys)))}
    sub_name = '/stereo_depth/point_cloud'
    def __init__(self):
        self.pub = rospy.Publisher('costmap', OccupancyGrid, queue_size=10)
        self.sub = rospy.Subscriber(self.sub_name, PointCloud2, queue_size=10)

    def unpack_points(self, msg):
        fields = len(msg.fields)
        l = len(msg.data)
        if msg.is_bigendian:
            control_str = ">"
        else:
            control_str = "<"
        for i in range(int(l/fields)):
            control_str += "f"
        data = struct.unpack(control_str, bytes(msg.data))
        return data

    def points_to_costmap(self, msg):
        data = self.unpack_points(msg)
        fields = len(msg.fields)
        da = np.array(data).reshape((int(len(data)/fields), fields))
        da.sort(axis = 0)
        cost_map = [self.costs[int(c)] if c != -1 else -1 for c in da[:, 3]]
        return cost_map

    def callback(self, msg):
        self.pub.data = self.points_to_costmap(msg)
        self.pub.publish()
        
    def node(self):
        rospy.spin()



if __name__ == '__main__':
    rospy.init_node('cloud_to_costmap_node', anonymous=True)
    c2m = cloud_to_costmap()
    try:
        c2m.node()
    except rospy.ROSInterruptException:
        pass
