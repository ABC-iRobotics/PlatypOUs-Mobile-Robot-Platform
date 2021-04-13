#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include <platypous_msgs/Convert.h>
#include <platypous_msgs/SendGoal.h>
#include <platypous_msgs/Pose2D.h>


static image_transport::Publisher image_pub;

static cv::Mat image;

static int map_width = 0;
static int map_height = 0;
static double map_resolution = 0.0;
static double map_origin_x = 0.0;
static double map_origin_y = 0.0;

static platypous_msgs::Pose2D robot_pose;

static ros::Publisher goal_pub;

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
}


platypous_msgs::Pose2D convert_map_to_image_coordinate(platypous_msgs::Pose2D input)
{
    platypous_msgs::Pose2D output;
    output.x = (-map_origin_x + input.x) / map_resolution;
    output.y = map_height - ((-map_origin_y + input.y) / map_resolution);
    output.yaw = input.yaw;
    return output;
}

platypous_msgs::Pose2D convert_image_to_map_coordinate(platypous_msgs::Pose2D input)
{
    platypous_msgs::Pose2D output;
    output.x = map_origin_x + (input.x * map_resolution);
    output.y = map_origin_y + ((map_height - input.y) * map_resolution);
    output.yaw = input.yaw;
    return output;
}


//~ bool convert_map_to_image_coordinate(platypous_msgs::Convert::Request &req, platypous_msgs::Convert::Response &res)
//~ {
    //~ res.output_x = (-map_origin_x + req.input_x) / map_resolution;
    //~ res.output_y = map_height - ((-map_origin_y + req.input_y) / map_resolution);
    //~ return true;
//~ }


//~ bool convert_image_to_map_coordinate(platypous_msgs::Convert::Request &req, platypous_msgs::Convert::Response &res)
//~ {
    //~ res.output_x = map_origin_x + (req.input_x * map_resolution);
    //~ res.output_y = map_origin_y + ((map_height - req.input_y) * map_resolution);
    //~ return true;
//~ }


bool send_nav_goal(platypous_msgs::SendGoal::Request &req, platypous_msgs::SendGoal::Response &res)
{
    geometry_msgs::PoseStamped goal;
    
    platypous_msgs::Pose2D map_goal = convert_image_to_map_coordinate(req.goal);
    
    goal.header.frame_id = "map";
    goal.pose.position.x = map_goal.x;
    goal.pose.position.y = map_goal.y;
    goal.pose.orientation.w = 1.0;
    
    goal_pub.publish(goal);
    
    res.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_to_image_node");
    ros::NodeHandle n;
    
    //~ ros::ServiceServer m_i = n.advertiseService("convert_map_to_image_coordinate", convert_map_to_image_coordinate);
    //~ ros::ServiceServer i_m = n.advertiseService("convert_image_to_map_coordinate", convert_image_to_map_coordinate);
    
    ros::ServiceServer send_goal_serv = n.advertiseService("send_nav_goal", send_nav_goal);
    
    image_transport::ImageTransport it(n);
    image_pub = it.advertise("output", 1);
    
    ros::Subscriber map_sub = n.subscribe("map", 10, map_callback);
    
    ros::Publisher robot_pose_map_pub = n.advertise<platypous_msgs::Pose2D>("robot_pose_map", 10);
    ros::Publisher robot_pose_img_pub = n.advertise<platypous_msgs::Pose2D>("robot_pose_image", 10);
    
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    
    tf::TransformListener listener;
    
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        tf::StampedTransform transform;
        
        try
        {
            listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
            robot_pose.x = transform.getOrigin().x();
            robot_pose.y = transform.getOrigin().y();

            double r, p;
            tf::Matrix3x3(transform.getRotation()).getEulerYPR(robot_pose.yaw, p, r);
        }
        catch(tf::TransformException ex){}
        
        image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg());
        
        robot_pose_map_pub.publish(robot_pose);
        robot_pose_img_pub.publish(convert_map_to_image_coordinate(robot_pose));
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
