#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/features/impl/normal_3d.hpp>

//rosrun sem_to_costmap sem_to_costmap
class points_to_map{
    public:
        nav_msgs::OccupancyGrid map_msg;
        sensor_msgs::PointCloud2 points_msg;
        bool ifGot = false;
        void myCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

};

struct obsts{
    int road = 0;
    int sidewalk = 1;
    int building = 2;
    int wall = 3;
    int fence = 4;
    int pole = 5;
    int traffic_light = 6;
    int traffic_sign = 7;
    int vegetation = 8;
    int terrain = 9;
    int sky = 10;
    int person = 11;
    int rider = 12;
    int car = 13;
    int truck = 14;
    int bus = 15;
    int train = 16;
    int motorcycle = 17;
    int bicycle = 18;
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
                pcl::PointCloud<pcl::PointXYZ> &pcl_cloud)
{
    //first convert from sensor_msgs to pcl_cpl2
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg_input, pcl_pc2);
    //then convert from pcl_pcl2 to 
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
}
// void calc_surface_Normals(pcl::PointCloud<pcl::PointXYZ> &input_cloud,
//                           pcl::PointCloud<pcl::_Normal> &normals)
// {
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::_Normal> ne;

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//     *cloud_ptr = input_cloud;
    
//     ne.setInputCloud(cloud_ptr);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//     ne.setSearchMethod(tree);
//     ne.setRadiusSearch(0.01);
//     ne.compute(normals);    
// }

// void generateMap(pcl::PointCloud<pcl::_Normal> normals, pcl::PointCloud<pcl::PointXYZ> cld,
//                float x_min, float x_max, float y_min, float y_max, std::vector<int> &map_points)
// {

//     for(int i = 0; i < cld.size(); i++)
//     {
//         float x = cld.points[i].x;
//         float y = cld.points[i].y;
//         float z = normals.points[i].normal_z;

//         float phi = acos(fabs(z));
//         float resolution = 1.0;
//         int xCell, yCell;
//         //if not NaN
//         if(z == z)
//         {
//             //calculating index of map array
//             xCell = (int)((x - x_min)/resolution);
//             yCell = (int)((y - y_min)/resolution);
//             map_points[xCell * yCell + xCell] ++;
//         }
//     }
// }

void generateMap(pcl::PointCloud<pcl::PointXYZ> cld, float x_min, float x_max,
                 float y_min, float y_max, std::vector<int8_t> &map_points)
{

    for(int i = 0; i < cld.size(); i++)
    {
        float x = cld.points[i].x;
        float y = cld.points[i].y;
        int obstacle_type = (int)(cld.points[i].data[3]);

        struct obsts o;
        float resolution = 1.0;

        //calculating index of map array
        int ix = (int)((x - x_min)/resolution);
        int iy = (int)((y - y_min)/resolution);
        int idx = ix * iy + ix;
        if(obstacle_type == o.road || obstacle_type == o.terrain)
            map_points[idx] = 50;
        else if(obstacle_type == o.sidewalk || obstacle_type == o.sky)
            map_points[idx] = 0;
        else
            map_points[idx] = 100;
    }
}
void calcMapSizes(float &x_max, float &y_max, float &x_min, float &y_min,
                                pcl::PointCloud<pcl::PointXYZ> cld)
{
    for(int i = 0; i < cld.size(); i++)
    {
        double x = cld.points[i].x;
        double y = cld.points[i].y;

        if(x_max < x)
            x_max = x;
        if(x_min > x)
            x_min = x;
        if(y_max < y)
            y_max = y;
        if(y_min > y)
            y_min = y;
    }
}


void updateGrid(nav_msgs::OccupancyGrid &msg, std::vector<int8_t> grid, int width, int height,
                                                                             float resolution)
{
    msg.header.seq ++;

    msg.header.stamp.sec = ros::Time::now().sec;
    msg.header.stamp.nsec = ros::Time::now().nsec;
    msg.info.map_load_time = ros::Time::now();

    msg.info.resolution = resolution;
    msg.info.width = width;
    msg.info.height = height;

    msg.data = grid;
}

int main(int argc, char **argv)
{
    std::string topic = std::string("/stereo_depth/point_cloud");
    //ROS_INFO("Starting node..");

    points_to_map ptmObject;
    
    ros::init(argc, argv, "sem_to_costmap");
    ros::NodeHandle n;

    float xMin, yMin = 1000.0;
    float xMax, yMax = -1000.0;
    


    ptmObject.map_msg.info.origin.position.x = 0;
    ptmObject.map_msg.info.origin.position.y = 0;
    ptmObject.map_msg.info.origin.position.z = 0;

    ptmObject.map_msg.info.origin.orientation.x = 0;
    ptmObject.map_msg.info.origin.orientation.y = 0;
    ptmObject.map_msg.info.origin.orientation.z = 0;
    ptmObject.map_msg.info.origin.orientation.w = 1;

    ptmObject.map_msg.header.frame_id = "map";
    ptmObject.map_msg.header.seq = 1;
    ptmObject.map_msg.header.stamp.sec = ros::Time::now().sec;
    ptmObject.map_msg.header.stamp.nsec = ros::Time::now().nsec; 
    
    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 1000, &points_to_map::myCallback, &ptmObject);
    //frequency in Herzs
    ros::Rate loop_rate(1000);
    //ROS_INFO("Starting loop..");
    while(ros::ok())
    {
        //ros::spinOnce() will call all the callbacks waiting to be called at that point in time.
        ros::spinOnce();
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
            pcl::PointCloud<pcl::PointXYZ> cld;
            pcl::PointCloud<pcl::_Normal> nrm;
            conversion(ptmObject.points_msg, cld);
            calcMapSizes(xMax, yMax, xMin, yMin, cld);
            //calc_surface_Normals(cld, nrm);
            std::vector<int8_t> _map(cld.size() * cld.size());
            int w, h = 0;
            float r = 0.0;
            generateMap(cld, xMax, yMax, xMin, yMin, _map);
            updateGrid(ptmObject.map_msg, _map, w, h, r);
            //ROS_INFO("Got normals to oZ: %.2f", nrm.points[0].normal_z);
            //ROS_INFO("Got vector:\nx %f, y %f, z %f, %f", cld.points[0].x, cld.points[0].y, cld.points[0].z, cld.points[0].data[3]);
            //ROS_INFO("Got vector:\nxmax %.2f, ymax %.2f, xmin %.2f, ymin %.2f", xMax, yMax, xMin, yMin);
            
            
            pub.publish(ptmObject.map_msg);
        }
        
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}




