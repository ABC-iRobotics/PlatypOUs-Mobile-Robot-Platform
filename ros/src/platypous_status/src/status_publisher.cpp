#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <platypous_msgs/PlatypousStatus.h>


static platypous_msgs::PlatypousStatus robot_status_msg;


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    robot_status_msg.robot_lin_vel = odom->twist.twist.linear.x;
    robot_status_msg.robot_ang_vel = odom->twist.twist.angular.z;
}

void voltage_callback(const std_msgs::Float64::ConstPtr& voltage)
{
    robot_status_msg.battery_voltage = voltage->data;
}

void current_callback(const std_msgs::Float64::ConstPtr& current)
{
    robot_status_msg.motor_current = current->data;
}

void status_callback(const std_msgs::String::ConstPtr& status)
{
    robot_status_msg.motor_driver_status = status->data;
}

void errors_callback(const std_msgs::String::ConstPtr& errors)
{
    robot_status_msg.motor_driver_errors = errors->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "status_publisher_node");
    ros::NodeHandle n;
    
    ros::Publisher robot_status_pub = n.advertise<platypous_msgs::PlatypousStatus>("robot_status", 10);
    
    ros::Subscriber odom_sub = n.subscribe("odom", 10, odom_callback);
    
    ros::Subscriber voltage_sub = n.subscribe("/motor_driver/status/voltage", 10, voltage_callback);
    ros::Subscriber current_sub = n.subscribe("/motor_driver/status/current", 10, current_callback);
    ros::Subscriber status_sub = n.subscribe("/motor_driver/status/status",  10, status_callback);
    ros::Subscriber errors_sub = n.subscribe("/motor_driver/status/errors",  10, errors_callback);
    
    tf::TransformListener listener;
    
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        tf::StampedTransform transform;
        
        try
        {
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            robot_status_msg.robot_pos_x = transform.getOrigin().x();
            robot_status_msg.robot_pos_y = transform.getOrigin().y();

            double r, p;
            tf::Matrix3x3(transform.getRotation()).getEulerYPR(robot_status_msg.robot_heading, p, r);
        }
        catch(tf::TransformException const&){}
        
        robot_status_pub.publish(robot_status_msg);
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
