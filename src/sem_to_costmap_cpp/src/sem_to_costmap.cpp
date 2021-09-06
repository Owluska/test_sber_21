#include "ros/ros.h"
// #include "rosbag/bag.h"
// #include "rosbag/view.h"

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"

sensor_msgs::PointCloud2 points_msg;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    points_msg = *msg;
}

int main(int argc, char **argv)
{
    std::string topic = std::string("/stereo_depth/point_cloud");
    nav_msgs::OccupancyGrid map_msg;
    ros::NodeHandle n;
    
    ros::init(argc, argv, "sem_to_costmap");

    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 1000, callback);

    ros::Rate loop(10);

    while(ros::ok())
    {
        pub.publish(map_msg);
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}



