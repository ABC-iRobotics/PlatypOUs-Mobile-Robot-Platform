#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Vector3.h>

#include "serialib/serialib.h"

serialib serial;

void rgb_callback(geometry_msgs::Vector3 msg)
{
    char str[30];
    sprintf(str, "%i,%i,%i\n", (uint32_t)msg.x, (uint32_t)msg.y, (uint32_t)msg.z);
    serial.writeString(str);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "led_driver_node");
    ros::NodeHandle n("~");
    
    if(serial.openDevice("/dev/ttyACM0", 115200) != 1)
    {
        printf("Unable to connect\n");
        return 1;
    }
    
    printf("Successful connection\n");
    
    ros::Subscriber rgb_sub = n.subscribe("rgb", 1, rgb_callback);

    ros::spin();
    
    serial.closeDevice();
    
    return 0;
}

