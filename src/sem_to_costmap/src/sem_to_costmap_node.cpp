#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>


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

pcl::PointCloud<pcl::PointXYZ>::Ptr conversion(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg_input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg_input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    return temp_cloud;
}
void calc_surface_Normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                          const pcl::PointCloud<pcl::Normal> normals)
{
    // //conversion from sensor_msgs::PointCloud2 to pcl::PointCloud2
    // pcl::PCLPointCloud2 pcl_pc2;
    // pcl_conversions::toPCL(*msg_input,pcl_pc2);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    //estimates local surface properties (surface normals and curvatures)at each 3D point
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::_Normal> ne;
    ne.setInputCloud(input_cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(normals);
}

int main(int argc, char **argv)
{
    std::string topic = std::string("/stereo_depth/point_cloud");
    //ROS_INFO("Starting node..");

    points_to_map ptmObject;
    
    ros::init(argc, argv, "sem_to_costmap");
    ros::NodeHandle n;


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
            
            auto raw_data = ptmObject.points_msg.data;
            uint8_t bytes_count = ptmObject.points_msg.fields[1].offset;
            std::vector<float> unpacked(raw_data.size()/4);
            bool isBigEndian = ptmObject.points_msg.is_bigendian;

            for(int i = 0; i < raw_data.back(); i++)
            {
                std::vector<uint32_t> buf(4);
                for(uint8_t j = 0; j < sizeof(float); j ++)
                {
                    buf[j] = raw_data[j+i];
                }
                float f = unpack(buf, isBigEndian);

                unpacked[i] = f;
                ROS_INFO("%.2f",f);
                i += bytes_count - 1;
            }

            ROS_INFO("Got vector with l: %d", unpacked.back());
            //pub.publish(ptmObject.map_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}




