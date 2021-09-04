from re import L
#! /usr/bin/env python3
import rosbag
import sensor_msgs.point_cloud2 as pc2
import struct
from time import sleep
import pandas as pd
path = "/home/kseniia/downloads/semantic_pseudo_point_cloud.bag"

bag = rosbag.Bag(path)

topic = "/stereo_depth/point_cloud"
#all fields has FLOAT32 datatype
d_bytes = 4

it = 0
for topic, msg, t in bag.read_messages(topics = topic):
    it += 1

    fields = len(msg.fields)
    l = len(msg.data)
    if msg.is_bigendian:
        control_str = ">"
    else:
        control_str = "<"
    for i in range(int(l/4)):
        control_str += "f"
    
    data = bytes(msg.data)
    data = struct.unpack(control_str, data)
    
    offset = int(msg.fields[3].offset/d_bytes) + 1
    #print(data)
    df = []
 
    for i, d in enumerate(data):
        if i % (offset) == 0 and i != 0 and i != (len(data) - 1):
            j = i - offset 
            #offset for x, y, z, color data
            row = [data[j + int(msg.fields[i].offset/d_bytes)] for i in range(fields)]
            df.append(row)
    print(it, end = " ")
    if it > 100:
        break

bag.close()
df = pd.DataFrame(df, columns = ['x', 'y', 'z','color'])
print(df.head(),"\n", df.shape)