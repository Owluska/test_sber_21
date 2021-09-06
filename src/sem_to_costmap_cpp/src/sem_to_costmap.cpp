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
        std::vector<uint8_t> data = points_msg.data;
        pub.publish(map_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}



