Vue.component("map-image", {
  template: `
  <div>
    <b-container fluid >
      <b-card body-text-variant="white" align="center" v-bind:style="{ backgroundColor: color_blue}">
        <b-card-text>
          Current goal:
          {{ click_x }}
          {{ click_y }}<br>
          Click on the map to give a goal:
          <b-button v-on:click="sendGoal" style="backgroundColor: #fab001;">
            <b-icon icon="play-fill" rounded style="width: 1.5em; height: 1.5em;"></b-icon>
          </b-button></p>
        </b-card-text>
      </b-card>
        
      <canvas id="map_canvas" 
              v-on:click="click"
              v-on:mousedown="mouseDown" 
              v-on:mouseup="mouseUp" 
              v-on:mousemove="mouseMove" 
              v-on:touchstart="touchDown" 
              v-on:touchend="touchUp" 
              v-on:touchmove="touchMove"
              width="850" 
              height="600" 
              style="
                border:1px solid #000000; 
                margin: auto; 
                padding: 0; 
                display: block;
                ">
      </canvas>
    </b-container>
  </div>`,
  
  data(){
    return {
      color_blue: '#1e2b4e',
      color_yellow: '#fab001',
      mouse_x: 0.0,
      mouse_y: 0.0,
      click_x: 0.0,
      click_y: 0.0,
      robot_pos_x: 0.0,
      robot_pos_y: 0.0,
      robot_yaw: 0.0,
      map_offset_x: 0.0,
      map_offset_y: 0.0,
      map_moving: false,
      map_canvas: null,
      ctx: null,
      img: new Image()
    };
  },
  
  mounted: function(){
    this.map_canvas = document.getElementById('map_canvas');
    this.ctx = map_canvas.getContext('2d');
  },
  
  methods: {
    renderMap: function(){
      this.ctx.fillStyle = "#000000";
      this.ctx.fillRect(0, 0, map_canvas.width, map_canvas.height);
      
      this.ctx.drawImage(this.img, this.map_offset_x, this.map_offset_y);
      
      this.ctx.beginPath();
      this.ctx.translate(this.robot_pos_x + this.map_offset_x, this.robot_pos_y + this.map_offset_y);
      this.ctx.rotate(-this.robot_yaw);
      this.ctx.rect(-10, -10, 20, 20);
      this.ctx.rotate(this.robot_yaw);
      this.ctx.translate(-(this.robot_pos_x + this.map_offset_x), -(this.robot_pos_y + this.map_offset_y));
      this.ctx.stroke();
      
      this.ctx.beginPath();
      this.ctx.arc(this.click_x + this.map_offset_x, this.click_y + this.map_offset_y, 10, 0, 2 * Math.PI);
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
    
    sendGoal: function () {
      socket.emit('navigation-goal', JSON.stringify({x: this.click_x, y: this.click_y}));
    },
    
    click: function (event) {
      this.click_x = event.offsetX - this.map_offset_x;
      this.click_y = event.offsetY - this.map_offset_y;
    },
    
    mouseDown: function (event) {
      this.mouse_x = event.offsetX - this.map_offset_x;
      this.mouse_y = event.offsetY - this.map_offset_y;
      this.map_moving = true;
    },
    
    mouseUp: function (event) {
      this.map_moving = false;
    },
    
    mouseMove: function (event) {
      if(this.map_moving){
        this.map_offset_x = event.offsetX - this.mouse_x;
        this.map_offset_y = event.offsetY - this.mouse_y;
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
