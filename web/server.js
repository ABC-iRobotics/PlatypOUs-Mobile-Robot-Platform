const app = require('express')();
var express = require('express');
const http = require('http').Server(app);
const io = require("socket.io")(http);
const rosnodejs = require('rosnodejs');
const navMsgs = rosnodejs.require('nav_msgs');
const geoMsgs = rosnodejs.require('geometry_msgs');
const ab2str = require('arraybuffer-to-string');

app.use(express.static("GUI"));

var cmd_vel_pub;
var nav_goal_pub;

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
  socket.on('ping', (msg) => {
    socket.emit('pong', null);
  });
});

rosnodejs.initNode('/my_node')
.then(() => {
    const nh = rosnodejs.nh;
        
    nh.subscribe("/depth_camera/color/image_raw/compressed", "sensor_msgs/CompressedImage", (msg) => {
      let base64Encoded = ab2str(msg.data, 'base64');
      io.emit("robot-camera", base64Encoded);
    });
    
    nh.subscribe("/map_image/compressed", "sensor_msgs/CompressedImage", (msg) => {
      let base64Encoded = ab2str(msg.data, 'base64');
      io.emit("map-image", base64Encoded);
    });
    
    nh.subscribe("/map_image_data", "platypous_msgs/MapImageData", (msg) => {
      io.emit("map-image-data", JSON.stringify(msg));
    });
    
    nh.subscribe("/driver/voltage", "std_msgs/Float64", (msg) => {
      io.emit("battery-voltage", msg.data);
    });
    
    cmd_vel_pub = nh.advertise('/cmd_vel/web_teleop', "geometry_msgs/Twist");
    nav_goal_pub = nh.advertise('/move_base_simple/goal', "geometry_msgs/PoseStamped");
  });
  
http.listen(3000, () => {
  console.log('listening on *:3000');
});
