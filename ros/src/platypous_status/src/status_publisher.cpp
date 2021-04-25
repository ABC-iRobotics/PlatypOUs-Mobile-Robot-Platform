#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <platypous_msgs/PlatypousStatus.h>


static platypous_msgs::PlatypousStatus robot_status_msg;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "status_publisher_node");
    ros::NodeHandle n;
    
    ros::Publisher robot_status_pub = n.advertise<platypous_msgs::PlatypousStatus>("robot_status", 10);
    
    tf::TransformListener listener;
    
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        tf::StampedTransform transform;
        
        try
        {
            listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
            robot_status_msg.robot_pos_x = transform.getOrigin().x();
            robot_status_msg.robot_pos_y = transform.getOrigin().y();

            double r, p;
            tf::Matrix3x3(transform.getRotation()).getEulerYPR(robot_status_msg.robot_heading, p, r);
        }
        catch(tf::TransformException ex){}
        
        robot_status_pub.publish(robot_status_msg);
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
