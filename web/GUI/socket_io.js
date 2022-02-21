var socket = io();

socket.on('robot-camera', function(data) {
  App.$refs.teleop_ref.renderImage(data);
});

socket.on('map-image', function(data) {
  App.$refs.navigation_ref.updateMap(data);
});

socket.on('map-image-data', function(data) {
  App.$refs.navigation_ref.updateMapData(data);
});

socket.on('pong', function(msg) {
  App.resetPingTimeout();
});

socket.on('robot-status', function(msg) {
  App.$refs.diagnostics_ref.updateRobotStatus(msg);
  App.$refs.navigation_ref.updateRobotStatus(msg);
  App.updateBatteryVoltage(msg.battery_voltage);
});

socket.on('sys-control-status', function(msg) {
  App.$refs.diagnostics_ref.updateSysStatus(msg);
});
