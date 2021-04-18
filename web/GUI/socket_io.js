var socket = io();

socket.on('robot-camera', function(data) {
  App.$refs.camera_img_ref.renderImage(data);
});

socket.on('robot-map', function(data) {
  App.$refs.slam_map_ref.updateMap(data);
});

socket.on('robot-pose', function(msg) {
  App.$refs.slam_map_ref.updateRobotPos(msg);
});

socket.on('pong', function(msg) {
  App.resetPingTimeout();
});
