const app = require('express')();
const http = require('http').Server(app);
const io = require("socket.io")(http);
const rosnodejs = require('rosnodejs');
const navMsgs = rosnodejs.require('nav_msgs');
const geoMsgs = rosnodejs.require('geometry_msgs');

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
});

rosnodejs.initNode('/my_node')
.then(() => {
    const nh = rosnodejs.nh;
    const sub = nh.subscribe('/localization/odometry/filtered', navMsgs.msg.Odometry, (msg) => {
      io.emit('x_pos', msg.pose.pose.position.x.toFixed(3));
      io.emit('y_pos', msg.pose.pose.position.y.toFixed(3));
      io.emit('lin_vel', msg.twist.twist.linear.x.toFixed(3));
      io.emit('ang_vel', msg.twist.twist.angular.z.toFixed(3));
    });
    
    pub = nh.advertise('/other/cmd_vel', geoMsgs.msg.Twist);
  });

http.listen(3000, () => {
  console.log('listening on *:3000');
});
