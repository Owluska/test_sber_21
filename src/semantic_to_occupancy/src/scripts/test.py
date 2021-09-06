#! /usr/bin/env python3

import rosbag
import struct
import numpy as np

path = "/home/kseniia/downloads/semantic_pseudo_point_cloud.bag"
topic = "/stereo_depth/point_cloud"

bag = rosbag.Bag(path)

def unpack_points(msg):
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


# def points_to_costmap(msg):
#     data = unpack_points(msg)

#     fields = len(msg.fields)
#     da = np.array(data).reshape((int(len(data)/fields), fields))
#     da.sort(axis = 0)
#     return cost_map

for topic, msg, t in bag.read_messages(topics = topic):   
    data = unpack_points(msg)
    fields = msg.fields
    print(data)
    print()
    print(fields)
    break

bag.close()