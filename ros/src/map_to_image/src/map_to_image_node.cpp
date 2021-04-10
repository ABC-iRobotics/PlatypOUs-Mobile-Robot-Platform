#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <map_to_image/Convert.h>


static image_transport::Publisher image_pub;

static cv::Mat image;

static int map_width = 0;
static int map_height = 0;
static double map_resolution = 0.0;
static double map_origin_x = 0.0;
static double map_origin_y = 0.0;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_width = msg->info.width;
    map_height = msg->info.height;
    map_resolution = msg->info.resolution;
    map_origin_x = msg->info.origin.position.x;
    map_origin_y = msg->info.origin.position.y;
    
    if(image.cols != map_width || image.rows != map_height)
    {
        image = cv::Mat::zeros(cv::Size(map_width, map_height), CV_8UC1);
    }
    
    for (int x = 0; x < map_width; x++)
    {
        for (int y = 0; y < map_height; y++)
        {
            int occupancy_value = msg->data[x + map_width * y];
            uint8_t image_value = 0;
            
            if (occupancy_value == -1)
            {
                image_value = 128;
            }
            else if (occupancy_value == 0)
            {
                image_value = 255;
            }
            else if (occupancy_value == 100)
            {
                image_value = 0;
            }
            
            image.at<uint8_t>(map_height - y - 1, x) = image_value;
        }
    }
    
    image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg());
}


bool convert_map_to_image_coordinate(map_to_image::Convert::Request &req, map_to_image::Convert::Response &res)
{
    res.output_x = (-map_origin_x + req.input_x) / map_resolution;
    res.output_y = map_height - ((-map_origin_y + req.input_y) / map_resolution);
    return true;
}


bool convert_image_to_map_coordinate(map_to_image::Convert::Request &req, map_to_image::Convert::Response &res)
{
    res.output_x = map_origin_x + (req.input_x * map_resolution);
    res.output_y = map_origin_y + ((map_height - req.input_y) * map_resolution);
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_to_image_node");
    ros::NodeHandle n;
    
    ros::ServiceServer m_i = n.advertiseService("convert_map_to_image_coordinate", convert_map_to_image_coordinate);
    ros::ServiceServer i_m = n.advertiseService("convert_image_to_map_coordinate", convert_image_to_map_coordinate);
    
    image_transport::ImageTransport it(n);
    image_pub = it.advertise("output", 1);
    
    ros::Subscriber map_sub = n.subscribe("map", 10, map_callback);
    
    
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
