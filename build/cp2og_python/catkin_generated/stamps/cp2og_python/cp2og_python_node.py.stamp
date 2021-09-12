#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid 
import struct
import numpy as np

class cloud_to_costmap():
    """
        Implementation of data transformation from sensor_msgs::PointCloud2
        to nav_msgs::OccupancyGrid (Map)
    """
    #Subscriber name
    sub_name = '/stereo_depth/point_cloud'
    #Obstacles types class
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
            self.motorcycle = 17
            self.bicycle = 18
    
    def __init__(self):
        #Obstacles class object
        self.obs = self.Obstacles()
        #Publisher variable 
        self.pub = rospy.Publisher('costmap', OccupancyGrid, queue_size=10)
        #Map massage container
        self.map_msg = OccupancyGrid()
        #Initialization of map messages container
        self.map_msg.info.origin.position.x = 0
        self.map_msg.info.origin.position.y = 0
        self.map_msg.info.origin.position.z = 0

        self.map_msg.info.origin.orientation.x = 0
        self.map_msg.info.origin.orientation.y = 0
        self.map_msg.info.origin.orientation.z = 0
        self.map_msg.info.origin.orientation.w = 1
        self.map_msg.header.frame_id = "map"
        #If data from Subscriber received then it is set True
        self.Got = False
        #PointCloud2 messages container
        self.cloud_msg = PointCloud2()
        #Subscriber variable     
        self.sub = rospy.Subscriber(self.sub_name, PointCloud2, self.callback, queue_size=10)
        #Loop rate
        self.rate = rospy.Rate(100)
        

    def unpack_points(self, msg):
        """
            This function unpackes bytes 1D-array
            to 1D-array of floats
        """
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
    
    def update_map_meta_data(self, data):
        """
            This functions updates meta data in
            map message container
        """
        x_max, y_max = np.max(data[:, :2], axis = 0)
        x_min, y_min = np.min(data[:, :2], axis = 0)
        resolution = 1.0

        self.map_msg.info.width = int((x_max - x_min)/resolution)
        self.map_msg.info.height = int((y_max - y_min)/resolution)
        self.map_msg.info.resolution = resolution

        self.map_msg.header.stamp = rospy.Time.now()

        self.map_msg.info.origin.position.x = -self.map_msg.info.width/2
        self.map_msg.info.origin.position.y = -self.map_msg.info.height/2
    
    def points_to_costmap(self):
        """
            This functions fills map with PointCloud data
            and fills all necessary fields of map message container
        """
        data = self.unpack_points(self.cloud_msg)

        fields = len(self.cloud_msg.fields)
        da = np.array(data).reshape((int(len(data)/fields), fields))

        self.update_map_meta_data(da)

        x_min, y_min = np.min(da[:, :2], axis = 0)
        cost_map = np.ones((self.map_msg.info.width * self.map_msg.info.height), dtype='int8') * 50

        print("raw {:d} floats {:d} XYZL {:d} w {:d} h {:d} map {:d}".format(len(self.cloud_msg.data),
        len(data), da.shape[0], self.map_msg.info.width, self.map_msg.info.height, cost_map.shape[0]))

        for d in da:
            x = d[0]
            y = d[1]
            c = int(d[3])

            ix = int((x - x_min)/self.map_msg.info.resolution) - 1
            iy = int((y - y_min)/self.map_msg.info.resolution) - 1
            i = int(ix * iy + ix)

            if cost_map[i] == 100 or cost_map[i] == 0 or i > cost_map.shape[0]:
                continue
            
            if c == self.obs.sky or c == self.obs.sidewalk:
                cost_map[i] = 0
            elif c == self.obs.terrain or c == self.obs.vegetation:
                continue
            else:
                cost_map[i] = 100 

        cost_map = list(cost_map)
        return cost_map

    def callback(self, msg):
        """
            Callback procedure for CloudPoint2 messages
        """ 
        self.Got = True
        self.cloud_msg = msg
    
        
    def node(self):
        """
            This function is waiting for data from Subscriber
            If it got data, it send message to Publisher
        """
        #While ROS ok
        while(not rospy.is_shutdown()):
            #If data from PointCloud received
            if self.Got:
                self.Got = False
                #Trying to transfrom data from PointCloud to OccupanceGrid
                try:
                    self.map_msg.data = self.points_to_costmap()
                #If something happend
                except Exception as e:
                    #If Crl+C pressed
                    if e == KeyboardInterrupt:
                        #Stop node
                        break
                    #Else sleeping and trying again
                    self.rate.sleep()
                    continue
                # If nothing happend, then puplishing and sleeping
                self.pub.publish(self.map_msg)
                self.rate.sleep()

if __name__ == '__main__':
    """
        This is main function
    """
    #Initialization of node
    rospy.init_node('cp2og_python_node', anonymous=True)
    #Creating class object
    c2m = cloud_to_costmap()
    #Trying to start node
    try:
        c2m.node()
    #If not sucseed, stopping
    except rospy.ROSInterruptException:
        pass