from re import L
#! /usr/bin/env python3
import rosbag
import sensor_msgs.point_cloud2 as pc2
import struct
import numpy as np
import pandas as pd
path = "/home/kseniia/downloads/semantic_pseudo_point_cloud.bag"

bag = rosbag.Bag(path)

topic = "/stereo_depth/point_cloud"
#all fields has FLOAT32 datatype
d_bytes = 4

it = 0

# fence - изгородь, pole - столб, vegetation - растительность
# terrain - местность
keys = ['road', 'sidewalk', 'building', 'wall','fence',
        'pole', 'traffic light', 'traffic sign', 'vegetation',
        'terrain', 'sky', 'person', 'rider', 'car', 'truck',
        'bus', 'train', 'motorcycle', 'bicycle']
classes = {k:i for k, i in zip(keys, range(len(keys)))}
obstacles = ['building', 'wall','fence','pole', 'traffic sign', 
             'person', 'rider', 'car', 'truck',
             'bus', 'train', 'motorcycle', 'bicycle']

free = ['sidewalk', 'sky']
fifty = ['road', 'traffic light']
seventy_five = ['vegetation', 'terrain']
#print(classes)
for topic, msg, t in bag.read_messages(topics = topic):
    it += 1

    fields = len(msg.fields)
    l = len(msg.data)
    if msg.is_bigendian:
        control_str = ">"
    else:
        control_str = "<"
    for i in range(int(l/fields)):
        control_str += "f"
    
    data = bytes(msg.data)
    data = struct.unpack(control_str, data)
    
    #offset = int(msg.fields[3].offset/d_bytes) + 1
    #print(data)
    #df = []

    #npa = np.zeros((int(len(data)/fields), fields))
    npa = np.array(data).reshape((int(len(data)/fields), fields))
    #print(npa.shape)
    # for i, d in enumerate(data):
    #     if i % (offset) == 0 and i != 0 and i != (len(data) - 1):
    #         j = i - offset 
    #         #offset for x, y, z, color data
    #         row = [data[j + int(msg.fields[i].offset/d_bytes)] for i in range(fields)]
    #         df.append(row)
    npa.sort(axis = 0)
    map = []
    for r in npa:
        for k, e in classes.items():
            color = int(r[3])
            if e == color:
                #print(e, color, k)
                if k in free:
                    map.append(0)
                elif k in obstacles:
                    map.append(100)
                elif k in fifty:
                    map.append(50)
                elif k in seventy_five:
                    map.append(75)
                else:
                    map.append(-1)
    if it > 1:
        print(map[:100])
        print(npa[:100])
        break

bag.close()
# print()
# df = pd.DataFrame(df, columns = ['x', 'y', 'z','color'])
# print(df.head())
