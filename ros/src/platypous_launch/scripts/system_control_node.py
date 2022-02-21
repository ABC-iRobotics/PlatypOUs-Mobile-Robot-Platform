#!/usr/bin/env python3


import rospy
import subprocess
import json

from std_msgs.msg import String


class SystemControlNode:
    
    process_handles = {}
    commands = None
    status = []
    
    def __init__(self):
        rospy.init_node("system_control_node")

        rospy.Subscriber("~command", String, self.command_callback, queue_size=2)

        status_pub = rospy.Publisher("~status", String, queue_size=2)
        status_msg = String()

        self.commands = rospy.get_param("~commands")

        for command in self.commands:
            self.status.append({"name": command["name"], "status": "stopped"})
            self.process_handles[command["name"]] = None

        rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            for ph in self.process_handles:
                if self.process_handles[ph] != None:
                    if self.process_handles[ph].poll() != None:
                        self.process_handles[ph] = None

                for st in self.status:
                    if st["name"] == ph:
                        if self.process_handles[ph] != None:
                            st["status"] = "running"
                        else:
                            st["status"] = "stopped"

            status_msg.data = json.dumps(self.status)
            status_pub.publish(status_msg)

            rate.sleep()
    
    def command_callback(self, msg):
        
        message = json.loads(msg.data)

        if message["name"] in self.process_handles.keys():
            if message["action"] == "start":
                if self.process_handles[message["name"]] == None:
                    cmd = ""
                    for command in self.commands:
                        if command["name"] == message["name"]:
                            cmd = command["command"]
                    self.process_handles[message["name"]] = subprocess.Popen("exec " + cmd, shell=True)
        
            if message["action"] == "stop":
                if self.process_handles[message["name"]] != None:
                    self.process_handles[message["name"]].terminate()


if __name__ == '__main__':
    node = SystemControlNode()
