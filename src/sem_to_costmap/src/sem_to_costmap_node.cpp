#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"

class points_to_map{
    public:
        nav_msgs::OccupancyGrid map_msg;
        sensor_msgs::PointCloud2 points_msg;
        void myCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

};

void points_to_map::myCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //TODO check if this right 
    this->points_msg = *msg;
}

int main(int argc, char **argv)
{
    std::string topic = std::string("/stereo_depth/point_cloud");
    // nav_msgs::OccupancyGrid map_msg;
    // sensor_msgs::PointCloud2 points_msg;
    ros::NodeHandle n;
    points_to_map ptmObject;
    
    
    ros::init(argc, argv, "sem_to_costmap");

    ptmObject.map_msg.info.origin.position.x = 0;
    ptmObject.map_msg.info.origin.position.y = 0;
    ptmObject.map_msg.info.origin.position.z = 0;

    ptmObject.map_msg.info.origin.orientation.x = 0;
    ptmObject.map_msg.info.origin.orientation.y = 0;
    ptmObject.map_msg.info.origin.orientation.z = 0;
    ptmObject.map_msg.info.origin.orientation.w = 1;

    ptmObject.map_msg.header.frame_id = "map";

    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 1000, &points_to_map::myCallback, &ptmObject);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        std::vector<uint8_t> raw_data = ptmObject.points_msg.data;
        
        int bytes_count = ptmObject.points_msg.fields[0].offset;
        //int data_length = sizeof(data)/sizeof(data[0]);
        int data_length = raw_data.size();

        std::vector<float> unpacked;
        bool isBigEndian = ptmObject.points_msg.is_bigendian;
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
        
        
        pub.publish(ptmObject.map_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}




