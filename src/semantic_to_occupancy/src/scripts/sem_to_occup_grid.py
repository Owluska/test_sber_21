from re import L
#! /usr/bin/env python3
import rosbag
import sensor_msgs.point_cloud2 as pc2
import pandas as pd
path = "/home/kseniia/downloads/semantic_pseudo_point_cloud.bag"

bag = rosbag.Bag(path)
ds = pd.DataFrame(columns = ["x", "y", "z", "color"])
#topic = "/stereo_depth/point_cloud"
#print(msg.fields, list(msg.data))
row = {"x":0.0, "y":0.0, "z":0.0, "color":0.0} 
for topic, msg, t in bag.read_messages():
    d = list(msg.data)
    print(len(d), msg.fields)
    # break
bag.close()
