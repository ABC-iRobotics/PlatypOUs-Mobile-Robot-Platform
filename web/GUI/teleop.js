Vue.component("teleop", {
  template: `
  <div 
    v-on:mouseup="stopDrawing" 
    v-on:mousemove="Draw" 
    v-on:keydown.u="Up"
    v-on:keydown.j="Down"
    v-on:keydown.h="Left"
    v-on:keydown.k="Right"
    v-on:keyup.u="stopDrawing"
    v-on:keyup.j="stopDrawing"
    v-on:keyup.h="stopDrawing"
    v-on:keyup.k="stopDrawing"
  >
    <b-container fluid>
      <b-row>
        <b-col >
          <b-card class="text-center" style="backgroundColor: #1e2b4e; color: #fab001;">
            <canvas id="camera_image" 
                    width="424" 
                    height="400"
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
        <b-col >
          <b-card class="text-center" style="backgroundColor: #1e2b4e; color: #fab001;">
            <b-card-text>
              Linear velocity: {{ velocities.lin }}
              Angular velocity: {{ velocities.ang }}
            </b-card-text>
            <canvas id="canvas"
                    v-on:mousedown="startDrawing" 
                    v-on:mouseup="stopDrawing" 
                    v-on:mousemove="Draw" 
                    v-on:touchstart="startDrawing" 
                    v-on:touchend="stopDrawing"
                    v-on:touchcancel="stopDrawing"
                    v-on:touchmove="Draw"
                    style="
                      display: block;
                      padding-left: 0;
                      padding-right: 0;
                      margin-left: auto;
                      margin-right: auto;
                      ">
            </canvas>
          </b-card>
        </b-col>
      </b-row>
    </b-container>
  </div>
  `,



  data(){
    return {
      lin_value: 0.7,
      ang_value: 1.2,
      velocities: {
        lin: 0.0,
        ang: 0.0
        //~ 640 x 360 --> szimulációs kamera kép
        //~ 848 x 800 --> valódi kamera kép
      },
      
      //~ camera
      img_camera: new Image(),
      ctx_camera: null,
      
      //~ joystick
      speed: 0.0,
      angle: 0.0,
      canvas: null,
      ctx: null,
      width: 0.0,
      height: 0.0,
      radius: 0.0,
      bg_radius: 0.0,
      mouse_x: 0.0,
      mouse_y: 0.0,
      joy_x: 0.0,
      joy_y: 0.0,
      angle_in_degrees: 0.0,
      is_joy_used: false,
      is_keyboard_used: false
    };
  },
  
  mounted: function(){
    this.width = 500;
    this.height = this.width;
    
    setInterval(this.sendVelocityCommand, 100);
    
    window.addEventListener('load', () => {
      
      this.canvas = document.getElementById('canvas');
      this.ctx = canvas.getContext('2d');
      
      this.resize();
      this.Draw();
      
      window.addEventListener('resize', this.resize);
    });
    
    //~ camera
    this.ctx_camera = document.getElementById('camera_image').getContext('2d');
    socket.emit('img_received');
  },
  
  methods: {
    
    //~ camera
    renderImage: function(image_data){
      this.img_camera.src = 'data:image/jpeg;base64, ' + image_data;
      
      this.ctx_camera.scale(0.5, 0.5);
      this.ctx_camera.drawImage(this.img_camera, 0, 0);
      this.ctx_camera.scale(2, 2);

      socket.emit('img_received');
    },
    
    resize: function(){
      //~ this.width = this.canvas.width;
      //~ this.height = this.canvas.width;
      this.canvas.height = this.height;
      this.canvas.width = this.width;
      
      this.joy_x = this.width / 2;
      this.joy_y = this.height / 2;
      this.mouse_x = this.width / 2;
      this.mouse_y = this.height / 2;
      this.bg_radius = this.width / 3;
      this.radius = this.bg_radius / 2;
      
      this.Draw();
    },
    
    startDrawing: function(event){
      this.is_joy_used = true;
      this.is_keyboard_used = true;
      this.Draw(event);
    },

    stopDrawing: function(event){
      this.mouse_x = this.width / 2;
      this.mouse_y = this.height / 2;
      this.velocities.lin = 0.0;
      this.velocities.ang = 0.0;
      
      this.sendVelocityCommand();
      this.is_joy_used = false;
      this.is_keyboard_used = false;
      this.Draw(event);
    },

    Draw: function(event){
      if (this.is_joy_used || this.is_keyboard_used)
      {
        var rect = this.canvas.getBoundingClientRect();
        
        this.mouse_x = event.clientX - rect.left || event.touches[0].clientX - rect.left;
        this.mouse_y = event.clientY - rect.top || event.touches[0].clientY - rect.top;
      }
        // clear canvas
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // background
        this.ctx.beginPath();
        this.ctx.arc(this.width / 2, this.height / 2, this.bg_radius, 0, Math.PI * 2, true);
        this.ctx.fillStyle = '#485987';
        this.ctx.fill();
        
        this.angle = Math.atan2((this.mouse_y - this.height / 2), (this.mouse_x - this.width / 2));

        if (Math.sign(this.angle) == -1) {
          this.angle_in_degrees = Math.round(-(this.angle) * 180 / Math.PI);
        }
        else {
          this.angle_in_degrees = Math.round( 360 - this.angle * 180 / Math.PI);
          if(this.angle_in_degrees == 360){
            this.angle_in_degrees = 0;
          }
        }

        if (Math.sqrt(Math.pow(this.mouse_x - this.width / 2, 2) + Math.pow(this.mouse_y - this.height / 2, 2)) <= this.bg_radius) {
          this.joy_x = this.mouse_x;
          this.joy_y = this.mouse_y;
          
        }
        else {
          this.joy_x = this.bg_radius * Math.cos(this.angle) + this.width / 2;
          this.joy_y = this.bg_radius * Math.sin(this.angle) + this.height / 2;
        }
        
        this.velocities.ang = ( this.ang_value * ((this.width / 2 - this.joy_x) / this.bg_radius)).toFixed(2);
        this.velocities.lin = ( this.lin_value * ((this.height / 2 - this.joy_y) / this.bg_radius)).toFixed(2);
        
        this.ctx.beginPath();
        this.ctx.arc(this.joy_x, this.joy_y, this.radius, 0, Math.PI * 2, true);
        this.ctx.fillStyle = '#fab001';
        this.ctx.fill();
        this.ctx.strokeStyle = '#c98e00';
        this.ctx.lineWidth = 10;
        this.ctx.stroke();

        this.speed =  Math.round(100 * Math.sqrt(Math.pow(this.joy_x - this.width / 2, 2) + 
                      Math.pow(this.joy_y - this.height / 2, 2)) / this.bg_radius);
    },
    
    Up: function(){
      this.velocities.lin = 0.35;
      this.velocities.ang = 0;
      console.log("asd");
      this.startDrawing();
    },
    
    Down: function(){
      this.velocities.lin = -0.35;
      this.velocities.ang = 0;
      this.startDrawing();
    },
    
    Left: function(){
      this.velocities.ang = 0.6;
      this.velocities.lin = 0;
      this.startDrawing();
    },
    
    Right: function(){
      this.velocities.ang = -0.6;
      this.velocities.lin = 0;
      this.startDrawing();
    },
    
    sendVelocityCommand: function(){
      if (this.is_joy_used || this.is_keyboard_used)
      {
        socket.emit('twist_message', JSON.stringify(this.velocities));
      }
    }
  }
});
