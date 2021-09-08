#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/features/impl/normal_3d.hpp>


class points_to_map{
    public:
        nav_msgs::OccupancyGrid map_msg;
        sensor_msgs::PointCloud2 points_msg;
        bool ifGot = false;
        void myCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

};

void points_to_map::myCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //TODO check if this right
    auto pntr = msg;
    if (pntr != nullptr) 
        this->points_msg = *pntr;
        this->ifGot = true;
}

float unpack(std::vector<uint32_t> bytes, bool isBigEndian)
{
    float f = 0.0;
    uint8_t* f_p = (uint8_t*)&f;
    uint8_t dl = sizeof(f);
    for(int i = 0; i < dl; i++)
    {
        if(isBigEndian)
            f_p[dl - i] = bytes[i];
        else
            f_p[i] = bytes[i];
    }
    return f;
}

void conversion(sensor_msgs::PointCloud2 msg_input,
pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud)
{
    //first convert from sensor_msgs to pcl_cpl2
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg_input, pcl_pc2);
    //then convert from pcl_pcl2 to 
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
}
void calc_surface_Normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                          pcl::PointCloud<pcl::_Normal>::Ptr normals)
{
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::_Normal> ne;
    ne.setInputCloud(input_cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);
}

void calcSize(double &xMax, double &yMax, double &xMin, double &yMin,
                                      pcl::PointCloud<pcl::PointXYZ> cld)
{
    for(int p = 0; p < cld.size(); p++)
    {
        double x = cld.points[p].x;
        double y = cld.points[p].y;
        if(xMax < x)
            xMax = x;
        if(xMin > x)
            xMin = x;
        if(yMax < y)
            yMax = y;
        if(yMin > y)
            yMin = y;
    }
}

int main(int argc, char **argv)
{
    std::string topic = std::string("/stereo_depth/point_cloud");
    //ROS_INFO("Starting node..");

    points_to_map ptmObject;
    
    ros::init(argc, argv, "sem_to_costmap");
    ros::NodeHandle n;

    double xMin,  yMin = 1000;
    double xMax,  yMax = -1000;


    ptmObject.map_msg.info.origin.position.x = 0;
    ptmObject.map_msg.info.origin.position.y = 0;
    ptmObject.map_msg.info.origin.position.z = 0;

    ptmObject.map_msg.info.origin.orientation.x = 0;
    ptmObject.map_msg.info.origin.orientation.y = 0;
    ptmObject.map_msg.info.origin.orientation.z = 0;
    ptmObject.map_msg.info.origin.orientation.w = 1;

    ptmObject.map_msg.header.frame_id = "map";
    
    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 10);
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 10, &points_to_map::myCallback, &ptmObject);

    ros::Rate loop_rate(10);
    //ROS_INFO("Starting loop..");
    while(ros::ok())
    {
        if (ptmObject.ifGot)
        {
            ptmObject.ifGot = false;
            
            // auto raw_data = ptmObject.points_msg.data;
            // uint8_t bytes_count = ptmObject.points_msg.fields[1].offset;
            // std::vector<float> unpacked(raw_data.size()/4);
            // bool isBigEndian = ptmObject.points_msg.is_bigendian;

            // for(int i = 0; i < raw_data.back(); i++)
            // {
            //     std::vector<uint32_t> buf(4);
            //     for(uint8_t j = 0; j < sizeof(float); j ++)
            //     {
            //         buf[j] = raw_data[j+i];
            //     }
            //     float f = unpack(buf, isBigEndian);

            //     unpacked[i] = f;
            //     ROS_INFO("%.2f",f);
            //     i += bytes_count - 1;
            // }
            // ROS_INFO("Got vector with l: %d", unpacked.back());
            pcl::PointCloud<pcl::PointXYZRGB> cld;
            conversion(ptmObject.points_msg, cld);


            ROS_INFO("Got vector with l: %d", cld.size());
            //pub.publish(ptmObject.map_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}




