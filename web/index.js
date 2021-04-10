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

io.on('connection', (socket) => {
  socket.on('twist_message', (msg) => {
    var twist = new geoMsgs.msg.Twist();
    twist.angular.z = msg;
    pub.publish(twist);
  });
  socket.on('goal', (msg) => {
    var goal = JSON.parse(msg);
    goal.x += 10;
    socket.emit('draw', JSON.stringify(goal));
  });
});

rosnodejs.initNode('/my_node')
.then(() => {
    const nh = rosnodejs.nh;
    
    const client = nh.serviceClient('/add_two_ints', 'beginner_tutorials/AddTwoInts');
    client.call({a: 1, b: 2});
    
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
    
    pub = nh.advertise('/platypous/cmd_vel', geoMsgs.msg.Twist);
  });
  
http.listen(3000, () => {
  console.log('listening on *:3000');
});
