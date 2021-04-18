var App = new Vue({
  el: '#app',
  data:{
    color_blue: '#1e2b4e',
    color_yellow: '#fab001',
    ping_timeout: 0
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
    }
  }
});

setInterval(App.sendPing, 200);
