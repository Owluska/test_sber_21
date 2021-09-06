#include "ros/ros.h"
// #include "rosbag/bag.h"
// #include "rosbag/view.h"

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"

sensor_msgs::PointCloud2 points_msg;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //TODO check if this right 
    points_msg = *msg;
}

int main(int argc, char **argv)
{
    std::string topic = std::string("/stereo_depth/point_cloud");
    nav_msgs::OccupancyGrid map_msg;
    ros::NodeHandle n;
    
    
    ros::init(argc, argv, "sem_to_costmap");

    map_msg.info.origin.position.x = 0;
    map_msg.info.origin.position.y = 0;
    map_msg.info.origin.position.z = 0;

    map_msg.info.origin.orientation.x = 0;
    map_msg.info.origin.orientation.y = 0;
    map_msg.info.origin.orientation.z = 0;
    map_msg.info.origin.orientation.w = 1;

    map_msg.header.frame_id = "map";

    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 1000, callback);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        std::vector<uint8_t> raw_data = points_msg.data;
        
        int bytes_count = points_msg.fields[0].offset;
        //int data_length = sizeof(data)/sizeof(data[0]);
        int data_length = raw_data.size();

        std::vector<float> unpacked;
        bool isBigEndian = points_msg.is_bigendian;
        //for(int i, j = 0; i < data_length, j < data_length - 4; i++, j += 4)
        for(auto i = raw_data.begin(); i != raw_data.end(); i++)
        {
            uint32_t d = 0;
            for(uint8_t j = 0; j < bytes_count; j++)
            {
                if(!isBigEndian)
                     d |= (*i << j); 
                else
                     d |= (*i >> j);         
            }
            std::cout << d;
            unpacked.at(*i) = d;
            i += bytes_count;
        }
        
        pub.publish(map_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}



