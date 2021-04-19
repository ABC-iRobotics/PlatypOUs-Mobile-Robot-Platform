const app = require('express')();
var express = require('express');
const http = require('http').Server(app);
const io = require("socket.io")(http);
const rosnodejs = require('rosnodejs');
const navMsgs = rosnodejs.require('nav_msgs');
const geoMsgs = rosnodejs.require('geometry_msgs');
const ab2str = require('arraybuffer-to-string');

app.use(express.static("GUI"));

var pub;
var client;
var client2;
var client3;

io.on('connection', (socket) => {
  socket.on('twist_message', (msg) => {
    var twist = new geoMsgs.msg.Twist();
    twist.angular.z = JSON.parse(msg).ang;
    twist.linear.x = JSON.parse(msg).lin;
    pub.publish(twist);
  });
  socket.on('navigation-goal', (msg) => {
    var goal = JSON.parse(msg);
    client.call({goal: {x: goal.x, y: goal.y} }).then((resp) => {
      console.log(resp.success);
    });
  });
  socket.on('ping', (msg) => {
    socket.emit('pong', null);
  });
});

rosnodejs.initNode('/my_node')
.then(() => {
    const nh = rosnodejs.nh;
    
    client = nh.serviceClient('/send_nav_goal', 'platypous_msgs/SendGoal');
    //~ client2 = nh.serviceClient('/convert_map_to_image_coordinate', 'platypous_msgs/Convert');
    //~ client3 = nh.serviceClient('/get_robot_pose', 'platypous_msgs/RobotPose');

    const sub = nh.subscribe('/localization/odometry/filtered', navMsgs.msg.Odometry, (msg) => {
      io.emit('x_pos', msg.pose.pose.position.x.toFixed(3));
      io.emit('y_pos', msg.pose.pose.position.y.toFixed(3));
      io.emit('lin_vel', msg.twist.twist.linear.x.toFixed(3));
      io.emit('ang_vel', msg.twist.twist.angular.z.toFixed(3));
    });
    
    nh.subscribe("/depth_camera/color/image_raw/compressed", "sensor_msgs/CompressedImage", (msg) => {
      let base64Encoded = ab2str(msg.data, 'base64');
      io.emit("robot-camera", base64Encoded);
    });
    
    nh.subscribe("/output/compressed", "sensor_msgs/CompressedImage", (msg) => {
      let base64Encoded = ab2str(msg.data, 'base64');
      io.emit("robot-map", base64Encoded);
    });
    
    nh.subscribe("/robot_pose_image", "platypous_msgs/Pose2D", (msg) => {
      io.emit("robot-pose", JSON.stringify(msg));
    });
    
    pub = nh.advertise('/cmd_vel/web_teleop', geoMsgs.msg.Twist);
  });
  
http.listen(3000, () => {
  console.log('listening on *:3000');
});
