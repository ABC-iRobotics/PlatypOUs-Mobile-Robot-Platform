#!/usr/bin/env python3

#import zmq
import random
import rospy
from geometry_msgs.msg import Twist
from paho.mqtt import client as mqtt_client

broker = '192.168.1.53'
port = 1883
topic = "test"
#generate client ID with pub prefix randomly 
client_id = f'python-mqtt-{random.randint(0,100)}'
#username = 'emqx'
# password = 'public'

pub = None

direction = 0


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client
    
def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        global direction
        data = msg.payload.decode()
        print(f"Received {msg.payload.decode()} from {msg.topic} topic")
        
        twist_msg = Twist()
        
        if(data == '1'):
            twist_msg.linear.x = 0.2
        if(data == '3'):
            twist_msg.linear.x = -0.2
        if(data == '2'):
            twist_msg.angular.z = -0.5
        if(data == '4'):
            twist_msg.angular.z = 0.5
        
        pub.publish(twist_msg)

    client.subscribe(topic)
    client.on_message = on_message

def run():
    global pub
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher ('/cmd_vel/eeg', Twist, queue_size=10)
    
    client = connect_mqtt()
    subscribe(client)

    while not rospy.is_shutdown():
        client.loop(.1)
    
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
    


#def listener():
#    context = zmq.Context()
#
#    print("Connecting to hello world server ...")
#    socket = context.socket(zmq.REQ)
#    socket.connect("tcp://192.168.1.168:8081")
#    socket.RCVTIMEO = 1000
#    pub = rospy.Publisher('chatter', String, queue_size=10)
#    rospy.init_node('talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#
#    request = 0
#    while not rospy.is_shutdown():
#        try:
#            print(f"Sending request {request} ...")
#            socket.send(b"Hello")
#            print("Waiting for response")
#            message = socket.recv()
#            rospy.loginfo(message)
#            pub.publish(str(message))
#            print("message: " + str(message))
#        except:
#            print("error")
#            socket = context.socket(zmq.REQ)
#            socket.connect("tcp://192.168.1.168:8081")
#            socket.RCVTIMEO = 1000
#
#        rate.sleep()
#        request = request + 1
#
#if __name__ == '__main__':
#    try:
#        listener()
#    except rospy.ROSInterruptException:
#        pass
