#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <auto_controller/Info.h>

#include "auto_controller.h"


static AutoController auto_cont;


static auto_controller::Info info_msg;


void odom_callback(const nav_msgs::OdometryConstPtr& odom)
{
    auto_cont.set_pose(odom->pose.pose.position.x, odom->pose.pose.position.y, std::atan2(2 * odom->pose.pose.orientation.w * odom->pose.pose.orientation.z, 1 - (2 * odom->pose.pose.orientation.z * odom->pose.pose.orientation.z)));
}

void set_target_angle_callback(const std_msgs::Float64ConstPtr& angle)
{
    auto_cont.set_target_angle(angle->data);
}

void set_target_speed_callback(const std_msgs::Float64ConstPtr& speed)
{
    auto_cont.set_target_speed(speed->data);
}

void set_target_point_callback(const geometry_msgs::PoseStampedConstPtr& target)
{
    auto_cont.set_target_point(target->pose.position.x, target->pose.position.y);
}

void set_on_callback(const std_msgs::BoolConstPtr& on)
{
    auto_cont.set_on(on->data);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_controller_node");
    ros::NodeHandle n("~");
    
    ros::Subscriber odom_sub = n.subscribe<const nav_msgs::OdometryConstPtr&>("odom", 10, odom_callback);
    
    ros::Subscriber set_target_angle_sub = n.subscribe<const std_msgs::Float64ConstPtr&>("set_target_angle", 10, set_target_angle_callback);
    ros::Subscriber set_target_speed_sub = n.subscribe<const std_msgs::Float64ConstPtr&>("set_target_speed", 10, set_target_speed_callback);
    ros::Subscriber set_target_point_sub = n.subscribe<const geometry_msgs::PoseStampedConstPtr&>("set_target_point", 10, set_target_point_callback);
    ros::Subscriber set_on_sub = n.subscribe<const std_msgs::BoolConstPtr&>("set_on", 10, set_on_callback);
    
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher info_pub = n.advertise<auto_controller::Info>("info", 10);
    
    AutoController::Config auto_controller_conf;
    n.param("angle_control_p_gain", auto_controller_conf.angle_control_p_gain, 1.0);
    n.param("angle_control_i_gain", auto_controller_conf.angle_control_i_gain, 0.0);
    n.param("angle_control_d_gain", auto_controller_conf.angle_control_d_gain, 0.0);
    n.param("angle_control_filter_time", auto_controller_conf.angle_control_filter_time, 0.0);
    n.param("cte_control_gain", auto_controller_conf.cte_control_gain, 0.0);
    n.param("cte_control_max_angle", auto_controller_conf.cte_control_max_angle, 0.0);
    n.param("max_speed", auto_controller_conf.max_speed, 0.1);
    n.param("max_ang_vel", auto_controller_conf.max_ang_vel, 0.5);
    n.param("target_point_min_distance", auto_controller_conf.target_point_min_distance, 0.5);
    auto_cont.set_config(auto_controller_conf);
    
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        
        auto_cont.update(0.01);
        
        geometry_msgs::Twist twist;
        twist.linear.x = auto_cont.get_lin_vel();
        twist.angular.z = auto_cont.get_ang_vel();
        cmd_vel_pub.publish(twist);
        
        info_msg.is_on = auto_cont.is_on();
        info_msg.current_x = auto_cont.get_current_x();
        info_msg.current_y = auto_cont.get_current_y();
        info_msg.current_angle = auto_cont.get_current_angle();
        info_msg.target_point_x = auto_cont.get_target_point_x();
        info_msg.target_point_y = auto_cont.get_target_point_y();
        info_msg.target_point_course = auto_cont.get_target_point_course();
        info_msg.target_point_angle = auto_cont.get_target_point_angle();
        info_msg.target_point_distance = auto_cont.get_target_point_distance();
        info_msg.target_point_course_deviation_angle = auto_cont.get_target_point_course_deviation_angle();
        info_msg.target_point_cross_track_error = auto_cont.get_target_point_cross_track_error();
        info_msg.target_angle = auto_cont.get_target_angle();
        info_msg.target_speed = auto_cont.get_target_speed();
        info_msg.lin_vel = auto_cont.get_lin_vel();
        info_msg.ang_vel = auto_cont.get_ang_vel();
        info_msg.control_target_angle = auto_cont.get_control_target_angle();
        info_msg.control_angle_error = auto_cont.get_control_angle_error();
        info_pub.publish(info_msg);
        
        loop_rate.sleep();
    }
    
    return 0;
}
