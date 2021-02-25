#!/usr/bin/env python3

import zmq
import rospy
from std_msgs.msg import String

def listener():
    context = zmq.Context()

    print("Connecting to hello world server ...")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://192.168.1.168:8081")
    socket.RCVTIMEO = 1000
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    request = 0
    while not rospy.is_shutdown():
        try:
            print(f"Sending request {request} ...")
            socket.send(b"Hello")
            print("Waiting for response")
            message = socket.recv()
            rospy.loginfo(message)
            pub.publish(str(message))
            print("message: " + str(message))
        except:
            print("error")
            socket = context.socket(zmq.REQ)
            socket.connect("tcp://192.168.1.168:8081")
            socket.RCVTIMEO = 1000

        rate.sleep()
        request = request + 1

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
