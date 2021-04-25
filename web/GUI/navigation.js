Vue.component("map-image", {
  template: `
  <div>
    <b-container fluid >
      <b-row>
        <b-col>
          <b-card style="backgroundColor: #1e2b4e; color: #fab001;">
            <b-row>
              <b-col cols="10">
                <b-card-text>
                  Current goal coordinates: 
                  X: {{ click_x }}
                  Y: {{ click_y }}<br>
                  Click on the map to give a goal!
                </b-card-text>
              </b-col>
              <b-col cols="2">
                <b-button v-on:click="sendGoal" style="backgroundColor: #fab001;">
                    <b-icon icon="play-fill" rounded style="width: 3em; height: 3em;"></b-icon>
                  </b-button>
              </b-col>
            </b-row>
          </b-card>
        </b-col>
      </b-row>
      <b-row>
        <b-col>
          <b-card style="backgroundColor: #1e2b4e; color: #fab001;">
            <canvas id="map_canvas" 
                    v-on:click="click"
                    v-on:mousedown="mouseDown" 
                    v-on:mouseup="mouseUp" 
                    v-on:mousemove="mouseMove" 
                    v-on:touchstart="touchDown" 
                    v-on:touchend="touchUp" 
                    v-on:touchmove="touchMove"
                    style="
                      display: block;
                      padding-left: 0;
                      padding-right: 0;
                      margin-left: auto;
                      margin-right: auto;
                      border:3px solid #fab001;
                    ">
            </canvas>
          </b-card>
        </b-col>
      </b-row>
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
    window.addEventListener('load', () => {
      
      this.map_canvas = document.getElementById('map_canvas');
      this.ctx = map_canvas.getContext('2d');
      this.resize();
      
      window.addEventListener('resize', this.resize);
    });
  },
  
  methods: {
    renderMap: function(){
      this.ctx.fillStyle = "#1e2b4e";
      this.ctx.fillRect(0, 0, this.map_canvas.width, this.map_canvas.height);
      
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
    
    resize: function(){
      this.map_canvas.width = 800;
      this.map_canvas.height = 800;
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
    },
    
    convert_map_to_image_coordinate: function(input_x, input_y){
      var output = new Object();
      output.x = (-map_origin_x + input.x) / map_resolution;
      output.y = map_height - ((-map_origin_y + input.y) / map_resolution);
      output.yaw = input.yaw;
      return output;
    },
    
    convert_image_to_map: function(){
      var output = new Object();
      output.x = map_origin_x + (input.x * map_resolution);
      output.y = map_origin_y + ((map_height - input.y) * map_resolution);
      output.yaw = input.yaw;
      return output;
    }
  }
});


//~ platypous_msgs::Pose2D convert_map_to_image_coordinate(platypous_msgs::Pose2D input)
//~ {
    //~ platypous_msgs::Pose2D output;
    //~ output.x = (-map_origin_x + input.x) / map_resolution;
    //~ output.y = map_height - ((-map_origin_y + input.y) / map_resolution);
    //~ output.yaw = input.yaw;
    //~ return output;
//~ }

//~ platypous_msgs::Pose2D convert_image_to_map_coordinate(platypous_msgs::Pose2D input)
//~ {
    //~ platypous_msgs::Pose2D output;
    //~ output.x = map_origin_x + (input.x * map_resolution);
    //~ output.y = map_origin_y + ((map_height - input.y) * map_resolution);
    //~ output.yaw = input.yaw;
    //~ return output;
//~ }
