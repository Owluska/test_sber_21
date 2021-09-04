from re import L
#! /usr/bin/env python3
import rosbag
import sensor_msgs.point_cloud2 as pc2
import struct
from time import sleep
import pandas as pd
path = "/home/kseniia/downloads/semantic_pseudo_point_cloud.bag"

bag = rosbag.Bag(path)
df = pd.DataFrame(columns = ["x", "y", "z", "color"])
topic = "/stereo_depth/point_cloud"
#print(msg.fields, list(msg.data))
row = {"x":0.0, "y":0.0, "z":0.0, "color":0.0}


#all fields has FLOAT32 datatype
d_bytes = 4
#amounts of fields
fields = 4
loc = 0
for topic, msg, t in bag.read_messages(topics = topic):
    l = len(msg.data)
    if msg.is_bigendian:
        control_str = ">"
    else:
        control_str = "<"
    for i in range(int(l/4)):
        control_str += "f"
    
    data = bytes(msg.data)
    data = struct.unpack(control_str, data)
    
    offset = int(msg.fields[3].offset/d_bytes)
    #df = []
 
    for i, d in enumerate(data):
        if i % (offset) == 0 and i != 0 and i != (len(data) - 1):
            j = i - offset

            #offset for x, y, z, color data
            xo, yo, zo, co = [int(msg.fields[i].offset/d_bytes) for i in range(fields)]

            row["x"] = data[j + xo]
            row["y"] = data[j + yo]
            row["z"] = data[j + zo]
            row["color"] = data[j + co]
            df.append(row, ignore_index=True)
    break

bag.close()
print(df.head())