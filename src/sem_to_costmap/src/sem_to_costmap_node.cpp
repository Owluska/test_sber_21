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
    private:
        sensor_msgs::PointCloud2 points_msg;
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
        }o;

        std::vector<float> float_point_data;
        std::vector<std::vector<float>> XYZLdata;
        std::vector<int8_t> map_data;


        float xMax = 0;
        float yMax = 0;
        float xMin = 0;
        float yMin = 0;

        int width = 0;
        int height = 0;
    
    public:
        float resolution = 1.0;
        bool ifGot = false;
        int map_size = 0;
        nav_msgs::OccupancyGrid map_msg;
        
        void myCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
        {
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

        void bytes_to_floats()
        {
            auto raw_data = this->points_msg.data;
            uint8_t bytes_count = this->points_msg.fields[1].offset;
            bool isBigEndian = this->points_msg.is_bigendian;
            
            size_t size = (size_t)(raw_data.size()/bytes_count);
            this->float_point_data.resize(size);

            for(int i = 0, u = 0; i < raw_data.size(); i += bytes_count, u++)
            {
                std::vector<uint8_t> buf(bytes_count);
                for(uint8_t j = 0; j < bytes_count; j ++)
                {
                    buf[j] = raw_data[j+i];
                }
                float f = unpack(buf, isBigEndian);

                //outp_data.push_back(f);
                this->float_point_data[u] = f;
                //ROS_INFO("A: %d %f %d", u, f, outp_data.size());
            }  
        }

        void  get_XYZL()
        {
            size_t cols = 4;
            size_t rows = this->float_point_data.size()/cols;
            
            this->XYZLdata.resize(rows, std::vector<float>(cols, 0.0));

            for(int i = 0; i < rows; i++)
            {
                
                std::vector<float> raw(4);
                for(int j = 0; j < cols; j++)
                {
                    this->XYZLdata[i][j] = this->float_point_data[i*cols + j];
                }
                //outp_data.push_back(raw);        
            }        
        }

        void preprocess_data()
        {
            bytes_to_floats();
            get_XYZL();
        }

        void calc_map_sizes()
        {
            for(int i = 0; i < this->XYZLdata.size(); i++)
            {
                float x = this->XYZLdata[i][0];
                float y = this->XYZLdata[i][1];

                if(this->xMax < x)
                    this->xMax = x;
                if(this->xMin > x)
                    this->xMin = x;
                if(this->yMax < y)
                    this->yMax = y;
                if(this->yMin > y)
                    this->yMin = y;
            }
            
            this->width = (int)((this->xMax - this->xMin)/this->resolution);
            this->height = (int)((this->yMax - this->yMin)/this->resolution);
            this->map_size = this->height * this->width;
        }

        void generate_map_data()
        {
            calc_map_sizes();
            this->map_data.resize(this->map_size, 50);
            ROS_INFO("%d", this->map_data.size());
            int idx = 0;
            for(int i = 0; i < this->XYZLdata.size(); i++)
            {
                float x = this->XYZLdata[i][0];
                float y = this->XYZLdata[i][1];
                int obstacle_type = (int)(this->XYZLdata[i][3]);

                //calculating index of map array
                int ix = (int)((x - this->xMin)/resolution);
                int iy = (int)((y - this->yMin)/resolution);
                idx = ix * iy + ix;
                //ROS_INFO("%d %d", map_points.size(), idx);
                    
                if(obstacle_type == this->o.road || obstacle_type == this->o.terrain)
                    this->map_data[idx] = 50;

                else if(obstacle_type == this->o.sidewalk || obstacle_type == this->o.sky)
                    this->map_data[idx] = 0;

                else
                    this->map_data[idx] = 100;

            }
        }

        void initialize_map()
        {
            this->map_msg.info.origin.position.x = 0;
            this->map_msg.info.origin.position.y = 0;
            this->map_msg.info.origin.position.z = 0;

            this->map_msg.info.origin.orientation.x = 0;
            this->map_msg.info.origin.orientation.y = 0;
            this->map_msg.info.origin.orientation.z = 0;
            this->map_msg.info.origin.orientation.w = 1;
            this->map_msg.info.resolution = 1;

            this->map_msg.header.frame_id = "map";
            this->map_msg.header.seq = 1;
            this->map_msg.header.stamp.sec = ros::Time::now().sec;
            this->map_msg.header.stamp.nsec = ros::Time::now().nsec; 
        }

        void update_map()
        {
            this->map_msg.header.seq ++;

            this->map_msg.header.stamp.sec = ros::Time::now().sec;
            this->map_msg.header.stamp.nsec = ros::Time::now().nsec;
            this->map_msg.info.map_load_time = ros::Time::now();

            this->map_msg.info.width = this->width;
            this->map_msg.info.height = this->height;

            this->map_msg.info.origin.position.x = -this->width / 2;
            this->map_msg.info.origin.position.y = -this->height / 2;

            this->map_msg.data = this->map_data;
        }

};







int main(int argc, char **argv)
{
    std::string topic = std::string("/stereo_depth/point_cloud");
    

    points_to_map ptmObject;
    
    
    //ROS_INFO("Starting node..");
    ros::init(argc, argv, "sem_to_costmap");    
    ros::NodeHandle n;
    //Cannot use ros::Time::now() before the first NodeHandle has been create
    ptmObject.initialize_map();  
    
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
                      
            ptmObject.preprocess_data();
            std::vector<int8_t> map_data; 
            
            
            ptmObject.generate_map_data();
            
            ptmObject.update_map();                     
            pub.publish(ptmObject.map_msg);
        }
        
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}




