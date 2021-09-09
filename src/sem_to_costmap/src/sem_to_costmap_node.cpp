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

void conversion(sensor_msgs::PointCloud2 input_msg,
                pcl::PointCloud<pcl::PointXYZ> &input_cloud)
{
    //first convert from sensor_msgs to pcl_cpl2
    pcl::PCLPointCloud2 pcl2_cloud;
    pcl_conversions::toPCL(input_msg, pcl2_cloud);
    //then convert from pcl_pcl2 to 
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl2_cloud, input_cloud);
}


void generateMap(pcl::PointCloud<pcl::PointXYZ> cld, float x_max, float y_max, float x_min,
                 float y_min, std::vector<int8_t> &map_points, float resolution)
{
    struct obsts o;
    for(int i = 0; i < cld.size(); i++)
    {
        float x = cld.points[i].x;
        float y = cld.points[i].y;
        int obstacle_type = (int)(cld.points[i].data[3]);
        //ROS_INFO("Obstacle_type %d", obstacle_type);



        //calculating index of map array
        int ix = (int)((x - x_min)/resolution);
        int iy = (int)((y - y_min)/resolution);
        int idx = ix * iy + ix;
               
        //ROS_INFO("Index %d %d %d", ix, iy, idx);
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


void updateGrid(nav_msgs::OccupancyGrid &msg, std::vector<int8_t> grid, int width, int height)
{
    msg.header.seq ++;

    msg.header.stamp.sec = ros::Time::now().sec;
    msg.header.stamp.nsec = ros::Time::now().nsec;
    msg.info.map_load_time = ros::Time::now();

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
    ptmObject.map_msg.info.resolution = 0.01;

    ptmObject.map_msg.header.frame_id = "map";
    ptmObject.map_msg.header.seq = 1;
    ptmObject.map_msg.header.stamp.sec = ros::Time::now().sec;
    ptmObject.map_msg.header.stamp.nsec = ros::Time::now().nsec; 
    
    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 1000, &points_to_map::myCallback, &ptmObject);
    //frequency in Herzs
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        //ros::spinOnce() will call all the callbacks waiting to be called at that point in time.
        ros::spinOnce();
        if (ptmObject.ifGot)
        {
            ptmObject.ifGot = false;
            
            pcl::PointCloud<pcl::PointXYZ> cld;
            pcl::PointCloud<pcl::_Normal> nrm;
            
            conversion(ptmObject.points_msg, cld);
            calcMapSizes(xMax, yMax, xMin, yMin, cld);


            
            
            float resolution = ptmObject.map_msg.info.resolution;
            int width = (int)((xMax - xMin)/resolution);
            int height = (int)((yMax - yMin)/resolution);

            int map_size = (int)(width * height);
            std::vector<int8_t> _map(map_size);

            generateMap(cld, xMax, yMax, xMin, yMin, _map, resolution);
            updateGrid(ptmObject.map_msg, _map, width, height);
                       
            pub.publish(ptmObject.map_msg);
            //ROS_INFO("Published map msg %d %d", width, height);
        }
        
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}




