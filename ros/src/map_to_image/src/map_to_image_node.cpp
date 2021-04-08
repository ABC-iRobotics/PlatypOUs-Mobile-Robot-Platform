#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


static image_transport::Publisher image_pub;

static cv::Mat image;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if(image.cols != msg->info.width || image.rows != msg->info.height)
    {
        image = cv::Mat::zeros(cv::Size(msg->info.width, msg->info.height), CV_8UC1);
    }
    
    for(int x = 0; x < image.cols; x++)
    {
        for(int y = 0; y < image.rows; y++)
        {
            image.at<uint8_t>(y, x) = (uint8_t)msg->data[x + image.cols * y];
        }
    }
    
    image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_to_image_node");
    ros::NodeHandle n;
    
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
