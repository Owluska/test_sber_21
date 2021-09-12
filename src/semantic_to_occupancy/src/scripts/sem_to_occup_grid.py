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
    # keys = [         'road',     'sidewalk',   'building',   'wall', 'fence',       'pole',
    #         'traffic light', 'traffic sign', 'vegetation','terrain',   'sky',     'person',
    #                 'rider',          'car',      'truck',    'bus', 'train', 'motorcycle', 
    #             'bicycle']
    # costs = [50,   0, 100, 100, 100, 100,
    #         100, 100,  50,  100, 0, 100,
    #         100, 100, 100, 100, 100, 100,
    #         100]
              
    
    # classes = {k:i for k, i in zip(keys, range(len(keys)))}
    sub_name = '/stereo_depth/point_cloud'

    class Obstacles:
        def __init__(self):
            self.road = 0
            self.sidewalk = 1
            self.building = 2
            self.wall = 3
            self.fence = 4
            self.pole = 5
            self.traffic_light = 6
            self.traffic_sign = 7
            self.vegetation = 8
            self.terrain = 9
            self.sky = 10
            self.person = 11
            self.rider = 12
            self.car = 13
            self.truck = 14
            self.bus = 15
            self.train = 16
            self.motorcycle =17
            self.bicycle = 18
    
    def __init__(self):
        self.pub = rospy.Publisher('costmap', OccupancyGrid, queue_size=10)
        self.msg = OccupancyGrid()
        self.obs = self.Obstacles()
        self.msg.info.origin.position.x = 0
        self.msg.info.origin.position.y = 0
        self.msg.info.origin.position.z = 0

        self.msg.info.origin.orientation.x = 0
        self.msg.info.origin.orientation.y = 0
        self.msg.info.origin.orientation.z = 0
        self.msg.info.origin.orientation.w = 1

        self.Got = False
        self.cloud_msg = PointCloud2()

        self.msg.header.frame_id = "map"

        self.sub = rospy.Subscriber(self.sub_name, PointCloud2, self.callback, queue_size=10)
        

    def unpack_points(self, msg):
        n_fields = len(msg.fields) 
        l = len(msg.data)
 
        if msg.is_bigendian:
            control_str = ">"
        else:
            control_str = "<"
        for i in range(int(l/n_fields)):
            control_str += "f"

        data = struct.unpack(control_str, bytes(msg.data))
        return data
    
    def get_meta_data(self, data):
        #l = data.shape[0]
        x_max, y_max = np.max(data[:, :2], axis = 0)
        x_min, y_min = np.min(data[:, :2], axis = 0)
        resolution = 1.0

        self.msg.info.width = int((x_max - x_min)/resolution)#int(math.sqrt(l))
        self.msg.info.height = int((y_max - y_min)/resolution)#int(math.sqrt(l))
        self.msg.info.resolution = resolution

        self.msg.header.stamp = rospy.Time.now()

        self.msg.info.origin.position.x = -self.msg.info.width/2
        self.msg.info.origin.position.y = -self.msg.info.height/2
    
    def points_to_costmap(self, msg):
        data = self.unpack_points(msg)

        fields = len(msg.fields)
        da = np.array(data).reshape((int(len(data)/fields), fields))

        da.sort(axis = 0)

        self.get_meta_data(da)

        x_min, y_min = np.min(da[:, :2], axis = 0)
        cost_map = np.ones((self.msg.info.width * self.msg.info.height), dtype='int8') * 50
        
        for d in da:
            x = d[0]
            y = d[1]
            c = int(d[3])

            ix = int((x - x_min)/self.msg.info.resolution) - 1
            iy = int((y - y_min)/self.msg.info.resolution) - 1
            i = int(ix * iy + ix)

            #print(c)
            if cost_map[i] == 100 or cost_map[i] == 0:
                continue
            
            if c == self.obs.sky or c == self.obs.sidewalk:
                cost_map[i] = 0
            elif c == self.obs.terrain or c == self.obs.vegetation:
                continue
            else:
                cost_map[i] = 100 
            #cost_map[i] = self.costs[int(c)]

        cost_map = list(cost_map)
        #cost_map = [self.costs[int(c)] if c != -1 else -1 for c in da[:, 3]]
        #rospy.loginfo("Data shape: {}, map shape: {}".format(da.shape, len(cost_map)))
        return cost_map

    def callback(self, msg): 
        self.Got = True
        self.cloud_msg = msg
    
        
    def node(self):
        while(not rospy.is_shutdown()):
            if self.Got:
                self.Got = False
                self.msg.data = self.points_to_costmap(self.cloud_msg)
                self.pub.publish(self.msg)





if __name__ == '__main__':
    rospy.init_node('cloud_to_costmap_node', anonymous=True)
    c2m = cloud_to_costmap()
    try:
        c2m.node()
    except rospy.ROSInterruptException:
        pass
