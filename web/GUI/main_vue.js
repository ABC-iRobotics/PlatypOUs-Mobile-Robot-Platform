var App = new Vue({
  el: '#app',
  data:{
    color_blue: '#1e2b4e',
    color_yellow: '#fab001',
    ping_timeout: 0,
    ping_sum: 0,
    ping_count: 0,
    ping_avg: 0,
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
    
    calculatePing: function(){
      this.ping_sum += this.ping_timeout;
      this.ping_count++;
      if ( this.ping_count == 100)
      {
        this.ping_avg = this.ping_sum / 100;
        this.ping_sum = 0;
        this.ping_count = 0;
      }
    },
    
    updateBatteryVoltage: function(voltage){
      this.battery_voltage = voltage.toFixed(2);
    }
  }
});

setInterval(App.sendPing, 200);
setInterval(App.calculatePing, 10);
