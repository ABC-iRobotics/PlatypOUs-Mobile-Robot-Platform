#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarSafetyNode:
    
    close_obstacle = False
    
    topic_timer = 0.0
    topic_timeout = 0.5
    is_timed_out = True
    
    frequency = 20.0
    
    def __init__(self):
        rospy.init_node("lidar_safety_node")
        
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback, queue_size=2)
        
        cmd_vel_pub = rospy.Publisher("/cmd_vel/web_teleop", Twist, queue_size=2)
        cmd_vel_msg = Twist()
        
        rate = rospy.Rate(self.frequency)
        
        while not rospy.is_shutdown():
            
            if (self.topic_timer > self.topic_timeout) or self.close_obstacle == True:
                cmd_vel_pub.publish(cmd_vel_msg)
            
            self.topic_timer += 1.0 / self.frequency

            rate.sleep()
    
    def laser_scan_callback(self, msg):
        self.topic_timer = 0.0
        
        for distance in msg.ranges:
            if distance < msg.range_max and distance > msg.range_min and distance < 0.5:
                self.close_obstacle = True
                return
        
        self.close_obstacle = False

if __name__ == '__main__':
    n = LidarSafetyNode()
