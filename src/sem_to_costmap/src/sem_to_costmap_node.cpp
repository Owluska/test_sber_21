#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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


struct PointXYZfL
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    float c;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN32;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZfL,
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, c, c)
)



void points_to_map::myCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //TODO check if this right
    auto pntr = msg;
    if (pntr != nullptr) 
        this->points_msg = *pntr;
        this->ifGot = true;
}

float unpack(std::vector<uint8_t> bytes, bool isBigEndian)
{
    float f = 0.0;
    uint8_t* f_p = (uint8_t*)&f;
    uint8_t dl = sizeof(f);
    for(int i = 0; i < dl; i++)
    {
        if(isBigEndian)
            f_p[dl - 1 - i] = bytes[i];
        else
            f_p[i] = bytes[i];
    }
    return f;
}

void conversion(sensor_msgs::PointCloud2 &input_msg,
                pcl::PointCloud<PointXYZfL> &input_cloud)
{
    //first convert from sensor_msgs to pcl_cpl2
    pcl::PCLPointCloud2 pcl2_cloud;
    pcl_conversions::toPCL(input_msg, pcl2_cloud);
    // for( int i = 0; i < pcl2_cloud.data.size(); i++)
    //     ROS_INFO("%d", pcl2_cloud.data[i]);
    //then convert from pcl_pcl2 to 
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl2_cloud, input_cloud);
}


void generateMap(pcl::PointCloud<PointXYZfL> cld, float x_max, float y_max, float x_min,
                 float y_min, std::vector<int8_t> &map_points, float resolution)
{
    struct obsts o;
    for(int i = 0; i < cld.size(); i++)
    {
        float x = cld.points[i].x;
        float y = cld.points[i].y;
        int obstacle_type = (int)(cld.points[i].c);
        //ROS_INFO("Obstacle_type %d", obstacle_type);
        //ROS_INFO("Got vector:\nx %f, y %f, z %f, %f", cld.points[0].x, cld.points[0].y, cld.points[0].z, cld.points[0].data[3]);



        //calculating index of map array
        int ix = (int)((x - x_min)/resolution);
        int iy = (int)((y - y_min)/resolution);
        int idx = ix * iy + ix;
               
        //ROS_INFO("Class value %d", obstacle_type);
        if(obstacle_type == o.road || obstacle_type == o.terrain)
            map_points[idx] = 50;
        else if(obstacle_type == o.sidewalk || obstacle_type == o.sky)
            map_points[idx] = 0;
        else
            map_points[idx] = 100;
    }

}
void calcMapSizes(float &x_max, float &y_max, float &x_min, float &y_min,
                                pcl::PointCloud<pcl::PointXYZL> cld)
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

void preprocess_data(points_to_map &Obj, std::vector<float> &outp_data)
{
    auto raw_data = Obj.points_msg.data;
    uint8_t bytes_count = Obj.points_msg.fields[1].offset;
    bool isBigEndian = Obj.points_msg.is_bigendian;
    
    size_t size = (size_t)(raw_data.size()/bytes_count);
    outp_data.resize(size);

    for(int i = 0, u = 0; i < raw_data.size(); i += bytes_count, u++)
    {
        std::vector<uint8_t> buf(bytes_count);
        for(uint8_t j = 0; j < bytes_count; j ++)
        {
            buf[j] = raw_data[j+i];
        }
        float f = unpack(buf, isBigEndian);

        //outp_data.push_back(f);
        outp_data[u] = f;
        //ROS_INFO("A: %d %f %d", u, f, outp_data.size());
    }  
}

void  getXYZL(std::vector<float> vec_data, std::vector<std::vector<float>> &outp_data)
{
    size_t cols = 4;
    size_t rows = vec_data.size()/cols;
    
    outp_data.resize(rows, std::vector<float>(cols, 0.0));

    for(int i = 0; i < rows; i++)
    {
        
        std::vector<float> raw(4);
        for(int j = 0; j < cols; j++)
        {
           outp_data[i][j] = vec_data[i*cols + j];
        }
        //outp_data.push_back(raw);        
    }        
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
            auto raw_data = ptmObject.points_msg.data;
            std::vector<float> pdata;
            
            preprocess_data(ptmObject, pdata);
            //ROS_INFO("A: %2.f %.2f %.2f %.2f", pdata[0], pdata[1], pdata[2], pdata[3]);

            std::vector<std::vector<float>> matrix;
            getXYZL(pdata, matrix);
            size_t rows = pdata.size()/4;
            // for(int i = 0; i < rows; i ++)
            // {
            //     ROS_INFO("A: %.2f %.2f %.2f %.2f", matrix[i][0], matrix[i][1], matrix[i][2], matrix[i][3]);
            // }

            //ROS_INFO("A: %.2f %.2f %.2f %.2f", pdata[4], pdata[5], pdata[6], pdata[7]); 

            
            // pcl::PointCloud<PointXYZfL> cld;
            
            // conversion(ptmObject.points_msg, cld);
            // ROS_INFO("x %.2f, y %.2f, z %.2f, %.2f", cld.points[1].x, cld.points[1].y, cld.points[1].z, cld.points[1].c);
            // for (int i = 0; i < cld.size(); i++)
            //     if(cld.points[i].data[3] != 1.0) 
            //         ROS_INFO("x %.2f, y %.2f, z %.2f, %.1f", cld.points[i].x, cld.points[i].y, cld.points[i].z, cld.points[i].data[3]);

            // calcMapSizes(xMax, yMax, xMin, yMin, cld);

            // float resolution = ptmObject.map_msg.info.resolution;
            // int width = (int)((xMax - xMin)/resolution);
            // int height = (int)((yMax - yMin)/resolution);

            // int map_size = (int)(width * height);
            // std::vector<int8_t> _map(map_size);

            // generateMap(cld, xMax, yMax, xMin, yMin, _map, resolution);
            // updateGrid(ptmObject.map_msg, _map, width, height);
                       
            // pub.publish(ptmObject.map_msg);
            //ROS_INFO("Published map msg %d %d", width, height);
        }
        
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}




