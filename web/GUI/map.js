Vue.component("map-image", {
  template: `
  <div>
    <label style="font-size: x-large; text-indent: 30pt; line-height: 50px;">
        Live map:
      </label><br/>
      
      <label style="font-size: large; text-indent: 30pt; line-height: 20px;">
        Current goal:
      </label><br/>
      
      <p style="font-size: large; text-indent: 30pt; line-height: 20px;">X: {{ mouse_x }}</p>
      <p style="font-size: large; text-indent: 30pt; line-height: 20px;">Y: {{ mouse_y }}</p>

      <label style="font-size: large; text-indent: 30pt; line-height: 20px;">
        Click on the map to give a goal:
      </label>

      <canvas id="map_canvas" 
              v-on:mousedown="mouseDown" 
              v-on:mouseup="mouseUp" 
              v-on:mousemove="mouseMove" 
              v-on:touchstart="touchDown" 
              v-on:touchend="touchUp" 
              v-on:touchmove="touchMove"
              width="720" height="720" 
              style="border:1px solid #000000; margin: auto; padding: 0; display: block;"></canvas>
  </div>`,
  
  data(){
    return {
      mouse_x: 0.0,
      mouse_y: 0.0,
      robot_pos_x: 0.0,
      robot_pos_y: 0.0,
      robot_yaw: 0.0,
      map_offset_x: 0.0,
      map_offset_y: 0.0,
      map_moving: false,
      ctx: null,
      img: new Image()
    };
  },
  
  mounted: function(){
    this.ctx = document.getElementById('map_canvas').getContext('2d');
  },
  
  methods: {
    renderMap: function(){
      this.ctx.fillStyle = "#000000";
      this.ctx.fillRect(0, 0, 720, 720);
      
      this.ctx.drawImage(this.img, this.map_offset_x, this.map_offset_y);
      
      this.ctx.beginPath();
      this.ctx.arc(this.robot_pos_x, this.robot_pos_y, 10, 0, 2 * Math.PI);
      this.ctx.stroke();
    },
    
    updateMap: function(image_data){
      this.img.src = 'data:image/jpeg;base64, ' + image_data;
      this.renderMap();
    },
    
    updateRobotPos: function(data){
      this.robot_pos_x = JSON.parse(data).x;
      this.robot_pos_y = JSON.parse(data).y;
      this.robot_yaw = JSON.parse(data).yaw;
      this.renderMap();
    },
    
    mouseDown: function (event) {
      this.mouse_x = event.clientX - this.map_offset_x;
      this.mouse_y = event.clientY - this.map_offset_y;
      this.map_moving = true;
    },
    
    mouseUp: function (event) {
      this.map_moving = false;
    },
    
    mouseMove: function (event) {
      if(this.map_moving){
        this.map_offset_x = event.clientX - this.mouse_x;
        this.map_offset_y = event.clientY - this.mouse_y;
        this.renderMap();
      }
    },
    
    touchDown: function (event) {
      this.mouse_x = event.touches[0].clientX - this.map_offset_x;
      this.mouse_y = event.touches[0].clientY - this.map_offset_y;
      this.map_moving = true;
    },
    
    touchUp: function (event) {
      this.map_moving = false;
    },
    
    touchMove: function (event) {
      if(this.map_moving){
        this.map_offset_x = event.touches[0].clientX - this.mouse_x;
        this.map_offset_y = event.touches[0].clientY - this.mouse_y;
        this.renderMap();
      }
    }
  }
});
