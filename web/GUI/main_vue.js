var App = new Vue({
  el: '#app',
  data:{
    color_blue: '#1e2b4e',
    color_yellow: '#fab001',
    ping_timeout: 0,
    ping_delay: 0,
    battery_voltage: 0.0,
    meta: {
      requiresAuth: true
    }
  },
  
  mounted: function(){
  },
  
  methods:{
    resetPingTimeout: function(){
      this.ping_timeout = 0;
    },
    
    sendPing: function(){
      this.ping_timeout++;
      socket.emit("ping", null);
    },
    
    updateBatteryVoltage: function(voltage){
      this.battery_voltage = voltage.toFixed(2);
    }
  }
});

setInterval(App.sendPing, 200);
