const app = require('express')();
const http = require('http').Server(app);
const io = require("socket.io")(http);
const rosnodejs = require('rosnodejs');
const navMsgs = rosnodejs.require('nav_msgs');
const geoMsgs = rosnodejs.require('geometry_msgs');
const ab2str = require('arraybuffer-to-string');

app.get('/', (req, res) => {
  res.sendFile(__dirname + '/index.html');
});

var pub;
var client;
var client2;
var client3;

io.on('connection', (socket) => {
  socket.on('twist_message', (msg) => {
    var twist = new geoMsgs.msg.Twist();
    twist.angular.z = msg;
    pub.publish(twist);
  });
  socket.on('goal', (msg) => {
    var goal = JSON.parse(msg);
    var mapx = 0, mapy = 0;
    client.call({input_x: goal.x, input_y: goal.y}).then((resp) => {
      client2.call({input_x: resp.output_x, input_y: resp.output_y}).then((resp) => { 
        
        var draw = new Object();
        draw.x = resp.output_x;
        draw.y = resp.output_y;
        socket.emit('draw', JSON.stringify(draw));
      
      });
    });
  });
});

rosnodejs.initNode('/my_node')
.then(() => {
    const nh = rosnodejs.nh;
    
    client = nh.serviceClient('/convert_image_to_map_coordinate', 'map_to_image/Convert');
    client2 = nh.serviceClient('/convert_map_to_image_coordinate', 'map_to_image/Convert');
    client3 = nh.serviceClient('/get_robot_pose', 'map_to_image/RobotPose');

    
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
      client3.call({input_x: 0.0, input_y: 0.0, input_yaw: 0.0}).then((resp) => {
          client2.call({input_x: resp.output_x, input_y: resp.output_y}).then((resp) => { 
        
            var draw = new Object();
            draw.x = resp.output_x;
            draw.y = resp.output_y;
            io.emit('draw', JSON.stringify(draw));
          
          });
      });
    });
    
    pub = nh.advertise('/platypous/cmd_vel', geoMsgs.msg.Twist);
  });
  
http.listen(3000, () => {
  console.log('listening on *:3000');
});
