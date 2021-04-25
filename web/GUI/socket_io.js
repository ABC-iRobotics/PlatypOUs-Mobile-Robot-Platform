var socket = io();

socket.on('robot-camera', function(data) {
  App.$refs.teleop_ref.renderImage(data);
});

socket.on('map-image', function(data) {
  App.$refs.slam_map_ref.updateMap(data);
});

socket.on('map-image-data', function(data) {
  App.$refs.slam_map_ref.updateMapData(data);
});

socket.on('pong', function(msg) {
  App.resetPingTimeout();
});

socket.on('battery-voltage', function(msg) {
  App.updateBatteryVoltage(msg);
});
