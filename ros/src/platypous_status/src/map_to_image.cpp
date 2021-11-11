#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include <platypous_msgs/MapImageData.h>


static cv::Mat image;

static platypous_msgs::MapImageData map_image_data_msg;

static image_transport::Publisher image_pub;
static ros::Publisher map_image_data_pub;


void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_image_data_msg.width = msg->info.width;
    map_image_data_msg.height = msg->info.height;
    map_image_data_msg.resolution = msg->info.resolution;
    map_image_data_msg.map_frame_image_x = (-msg->info.origin.position.x) / msg->info.resolution;
    map_image_data_msg.map_frame_image_y = msg->info.height - (-msg->info.origin.position.y) / msg->info.resolution;
    
    if((uint32_t)image.cols != msg->info.width || (uint32_t)image.rows != msg->info.height)
    {
        image = cv::Mat::zeros(cv::Size(msg->info.width, msg->info.height), CV_8UC1);
    }
    
    for (uint32_t x = 0; x < msg->info.width; x++)
    {
        for (uint32_t y = 0; y < msg->info.height; y++)
        {
            int occupancy_value = msg->data[x + msg->info.width * y];
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
            
            image.at<uint8_t>(msg->info.height - y - 1, x) = image_value;
        }
    }
    
    
    image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg());
    
    map_image_data_pub.publish(map_image_data_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_to_image_node");
    ros::NodeHandle n;
    
    image_transport::ImageTransport it(n);
    image_pub = it.advertise("map_image", 1);
    
    map_image_data_pub = n.advertise<platypous_msgs::MapImageData>("map_image_data", 10);
    
    ros::Subscriber map_sub = n.subscribe("map", 10, map_callback);
    
    ros::spin();
    
    return 0;
}
