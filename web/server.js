const app = require('express')();
var express = require('express');
const http = require('http').Server(app);
const io = require("socket.io")(http);
const rosnodejs = require('rosnodejs');
const geoMsgs = rosnodejs.require('geometry_msgs');
const stdMsgs = rosnodejs.require('std_msgs');
const ab2str = require('arraybuffer-to-string');

app.use(express.static("GUI"));

var cmd_vel_pub;
var nav_goal_pub;
var system_control_command_pub;

var send_img = true;

io.on('connection', (socket) => {
  socket.on('twist_message', (msg) => {
    var twist = new geoMsgs.msg.Twist();
    twist.angular.z = JSON.parse(msg).ang;
    twist.linear.x = JSON.parse(msg).lin;
    cmd_vel_pub.publish(twist);
  });
  socket.on('navigation-goal', (msg) => {
    var pose = new geoMsgs.msg.PoseStamped();
    pose.pose.position.x = JSON.parse(msg).x;
    pose.pose.position.y = JSON.parse(msg).y;
    pose.pose.orientation.w = 1;
    pose.header.frame_id = "map";
    nav_goal_pub.publish(pose);
  });
  socket.on("system_control", (msg) => {
    var message = new stdMsgs.msg.String();
    message.data = msg;
    system_control_command_pub.publish(message);
  });
  socket.on('ping', (msg) => {
    socket.emit('pong', null);
  });
  socket.on("img_received", (err) => {
    send_img = true;
  });
  socket.on("connect_error", (err) => {
    console.log(`connect_error due to ${err.message}`);
  });
});

rosnodejs.initNode('/my_node')
.then(() => {
    const nh = rosnodejs.nh;
        
    nh.subscribe("/camera/color/image_raw/compressed", "sensor_msgs/CompressedImage", (msg) => {
      if(send_img){
        let base64Encoded = ab2str(msg.data, 'base64');
        io.emit("robot-camera", base64Encoded);
        send_img = false;
      }
    });
    
    nh.subscribe("/map_image/compressed", "sensor_msgs/CompressedImage", (msg) => {
      let base64Encoded = ab2str(msg.data, 'base64');
      io.emit("map-image", base64Encoded);
    });
    
    nh.subscribe("/map_image_data", "platypous_msgs/MapImageData", (msg) => {
      io.emit("map-image-data", JSON.stringify(msg));
    });

    nh.subscribe("/robot_status", "platypous_msgs/PlatypousStatus", (msg) => {
      io.emit("robot-status", msg);
    });

    nh.subscribe("/system_control_node/status", "std_msgs/String", (msg) => {
      io.emit("sys-control-status", JSON.parse(msg.data));
    });
    
    cmd_vel_pub = nh.advertise('/cmd_vel/web_teleop', "geometry_msgs/Twist");
    nav_goal_pub = nh.advertise('/move_base_simple/goal', "geometry_msgs/PoseStamped");
    system_control_command_pub = nh.advertise('/system_control_node/command', "std_msgs/String");
  });
  
http.listen(3000, () => {
  console.log('listening on *:3000');
});
