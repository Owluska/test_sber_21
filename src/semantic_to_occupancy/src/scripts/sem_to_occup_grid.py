#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid 
import struct
import numpy as np
import math
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
            50, 100,  50,  100, 100, 100,
            100, 100, 100, 100, 100, 100,
            100]
    # costs = [127,   0, 255, 255, 255, 255,
    #     127, 255,  127,  127,   0, 255,
    #     255, 255, 255, 255, 255, 255,
    #     255]               
    
    classes = {k:i for k, i in zip(keys, range(len(keys)))}
    sub_name = '/stereo_depth/point_cloud'
    def __init__(self):
        self.pub = rospy.Publisher('costmap', OccupancyGrid, queue_size=10)
        self.msg = OccupancyGrid()
        self.msg.info.origin.position.x = 0
        self.msg.info.origin.position.y = 0
        self.msg.info.origin.position.z = 0

        self.msg.info.origin.orientation.x = 0
        self.msg.info.origin.orientation.y = 0
        self.msg.info.origin.orientation.z = 0
        self.msg.info.origin.orientation.w = 1

        self.msg.header.frame_id = "map"

        self.sub = rospy.Subscriber(self.sub_name, PointCloud2, self.callback, queue_size=10)
        

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
    
    def get_meta_data(self, msg, data):
        l = data.shape[0]
        self.msg.info.width = int(math.sqrt(l))
        self.msg.info.height = int(math.sqrt(l))
 
        self.msg.info.resolution = 0.002
    
    def points_to_costmap(self, msg):
        data = self.unpack_points(msg)

        fields = len(msg.fields)
        da = np.array(data).reshape((int(len(data)/fields), fields))
        da.sort(axis = 0)

        self.get_meta_data(msg, da)
        cost_map = [self.costs[int(c)] if c != -1 else -1 for c in da[:, 3]]
        #rospy.loginfo("Data shape: {}, map shape: {}".format(da.shape, len(cost_map)))
        return cost_map

    def callback(self, msg):
        self.msg.data = self.points_to_costmap(msg)
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)
        
    def node(self):
        rospy.spin()



if __name__ == '__main__':
    rospy.init_node('cloud_to_costmap_node', anonymous=True)
    c2m = cloud_to_costmap()
    try:
        c2m.node()
    except rospy.ROSInterruptException:
        pass
