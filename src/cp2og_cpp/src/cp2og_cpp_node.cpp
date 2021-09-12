#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"

/**
 * Implementation of data transformation from sensor_msgs::PointCloud2
 * to nav_msgs::OccupancyGrid (Map)
 *
 */
class points_to_map{
    private:
        /// Container for ROS PointCloud2 message.
        sensor_msgs::PointCloud2 points_msg;
        /// Structure saving classes numbers for obstacles classes
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
        ///Vector for saving 1D-float PointCloud2 data
        std::vector<float> float_point_data;
        ///Vector for saving 2D-float PointCloud2 data
        std::vector<std::vector<float>> XYZLdata;
        ///Vector for saving ROS nav_msgs::OccupancyGrid map data
        std::vector<int8_t> map_data;

        ///Maximum x-value in PointCloud or Map data
        float xMax = 0;
        ///Maximum y-value in PointCloud or Map data
        float yMax = 0;
        ///Minimum x-value in PointCloud or Map data
        float xMin = 0;
        ///Minimum y-value in PointCloud or Map data
        float yMin = 0;

        ///Map width variable
        int map_width = 0;
        ///Map height variable
        int map_height = 0;
        ///Map size variable
        int map_size = 0;
        ///Map resolution variable
        float map_resolution = 1.0;
    
    public:
        /// Flag is set if data was recieved from sensor_msgs::PointCloud2 topic.
        bool ifGot = false;
        /// Map message for ROS nav_msgs::OccupancyGrid topic 
        nav_msgs::OccupancyGrid map_msg;

        
        /**
         * sensor_msgs::PointCloud2 subscriber callback procedure
         * @param - msg Pointer to ROS PointCloud2 message
         */
        void myCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
        {
            auto pntr = msg;
            if (pntr != nullptr) 
                this->points_msg = *pntr;
                this->ifGot = true;
        }

        /**
         * Procedure for convertation 4 unsigned byte numbers to 1 float number.
         * @param - bytes - Input bytes array 
         * @param - isBigEndian - Bytes order flag, if order is Big Endian - equals True
         */
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

        /**
         * Reshapes data from std::vector<uint8_t> to std::vector<uint8_t>.
         */
        void bytes_to_floats()
        {
            auto raw_data = this->points_msg.data;
            uint8_t bytes_count = this->points_msg.fields[1].offset;
            bool isBigEndian = this->points_msg.is_bigendian;
            
            size_t size = (size_t)(raw_data.size()/bytes_count);
            this->float_point_data.resize(size, 0);

            for(int i = 0, u = 0; i < raw_data.size(); i += bytes_count, u++)
            {
                std::vector<uint8_t> buf(bytes_count);
                for(uint8_t j = 0; j < bytes_count; j ++)
                {
                    buf[j] = raw_data[j+i];
                }
                float f = unpack(buf, isBigEndian);
                this->float_point_data[u] = f;
            }  
        }
        
        /**
         * Reshapes data from std::vector<float> to std::vector<std::vector<float>>.
         */
        void reshape_floats()
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
            }        
        }

        void preprocess_data()
        {
            bytes_to_floats();
            reshape_floats();
        }

        /**
         * Calculates minimum and maximum x and y values of PointCloud2 data.
         */
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
            
            this->map_width = (int)((this->xMax - this->xMin)/this->map_resolution);
            this->map_height = (int)((this->yMax - this->yMin)/this->map_resolution);
            this->map_size = this->map_height * this->map_width;
        }
        
        /**
         * This procedure filling map with data from CloudPoint2.
         */
        void generate_map_data()
        {
            calc_map_sizes();
            this->map_data.resize(this->map_size, 50);
            ROS_INFO("raw %d floats %d XYZL %d w %d h %d map %d", this->points_msg.data.size(),\
            this->float_point_data, this->XYZLdata.size(), this->map_width, this->map_height, this->map_data.size());
            int idx = 0;
            for(int i = 0; i < this->XYZLdata.size(); i++)
            {
                float x = this->XYZLdata[i][0];
                float y = this->XYZLdata[i][1];
                int obstacle_type = (int)(this->XYZLdata[i][3]);

                int ix = (int)((x - this->xMin)/map_resolution) - 1;
                int iy = (int)((y - this->yMin)/map_resolution) - 1;
                idx = ix * iy + ix;
                    
                if(this->map_data[idx] == 100 || this->map_data[idx] == 0)
                    continue;

                if(obstacle_type == this->o.road || obstacle_type == this->o.terrain)
                    this->map_data[idx] = 50;

                else if(obstacle_type == this->o.sidewalk || obstacle_type == this->o.sky)
                    this->map_data[idx] = 0;

                else
                    this->map_data[idx] = 100;

            }
        }
        /**
         * This procedure initialazing map message container.
         */
        void initialize_map()
        {
            this->map_msg.info.origin.position.x = 0;
            this->map_msg.info.origin.position.y = 0;
            this->map_msg.info.origin.position.z = 0;

            this->map_msg.info.origin.orientation.x = 0;
            this->map_msg.info.origin.orientation.y = 0;
            this->map_msg.info.origin.orientation.z = 0;
            this->map_msg.info.origin.orientation.w = 1;
            this->map_msg.info.resolution = this->map_resolution;

            this->map_msg.header.frame_id = "map";
            this->map_msg.header.seq = 1;
            this->map_msg.header.stamp.sec = ros::Time::now().sec;
            this->map_msg.header.stamp.nsec = ros::Time::now().nsec; 
        }
        /**
         * This procedure updates data in ROS map message container.
         */
        void update_map()
        {
            this->map_msg.header.seq ++;

            this->map_msg.header.stamp.sec = ros::Time::now().sec;
            this->map_msg.header.stamp.nsec = ros::Time::now().nsec;
            this->map_msg.info.map_load_time = ros::Time::now();

            this->map_msg.info.width = this->map_width;
            this->map_msg.info.height = this->map_height;

            this->map_msg.info.origin.position.x = -this->map_width / 2;
            this->map_msg.info.origin.position.y = -this->map_height / 2;

            this->map_msg.data = this->map_data;
        }

};


/**
 * Main function
 */
int main(int argc, char **argv)
{
    ///Subscriber topic name
    std::string topic = std::string("/stereo_depth/point_cloud");
    /// points_to_map class object
    points_to_map ptmObject;
    
    //.......1.Starting ROS node
    ros::init(argc, argv, "sem_to_costmap"); 
    /// ROS node handler variable   
    ros::NodeHandle n;
    //.......2.Initializing map message
    ptmObject.initialize_map();  
    
    ///Publisher topic variable creating and initialization
    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1000);
    ///Subscriber topic variable creating and initialization
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 1000, &points_to_map::myCallback, &ptmObject);
    ///Loop rate variable creating and initialization
    ros::Rate loop_rate(1000);
    //.......3.If ROS - ok, loop
    while(ros::ok())
    {
        //ros::spinOnce() will call all the callbacks waiting to be called at that point in time.
        ros::spinOnce();
        //.......4.Check if data from subscriber recieved
        if (ptmObject.ifGot)
        {
            //.......5.Reseting flag
            ptmObject.ifGot = false;
            //.......6.Transforming data from 1D bytes vector  to 2D floats vector          
            ptmObject.preprocess_data();
            //.......7.Filling map            
            ptmObject.generate_map_data();
            //.......8.Updating map message           
            ptmObject.update_map(); 
            //.......9.Publishing map message       
            pub.publish(ptmObject.map_msg);
        }
        //.......10.Sleeping while loop_rate
        loop_rate.sleep();
    }
    return 0;
}




