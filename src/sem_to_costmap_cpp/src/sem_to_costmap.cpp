#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"

#include <boost/foreach.hpp>


// const char path[100] = "/home/kseniia/downloads/semantic_pseudo_point_cloud.bag";
rosbag::Bag bag;
std::string path, topic;

int main(int argc, char **argv)
{
    path = std::string("/home/kseniia/downloads/semantic_pseudo_point_cloud.bag");
    topic = std::string("/stereo_depth/point_cloud");
    bag.open(path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    
    for(rosbag::MessageInstance const m : view)
    {
        m.getTopic();
        sensor_msgs::PointCloud2::ConstPtr msg_p = m.instantiate<sensor_msgs::PointCloud2>();
        // if(msg_p != nullptr)
        // {
        //     //std::cout << msg_p->data << std::endl;
        // }
        // return;
    }
    bag.close();
}



