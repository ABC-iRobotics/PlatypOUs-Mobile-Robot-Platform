Vue.component("teleop", {
  template: `
  <div>
    <b-container fluid>
    
    <b-card>
        <b-card-text>
          X: {{ x_coordinate }}
          Y: {{ y_coordinate }}
          Speed: {{ speed }} %
          Angle: {{ angle }}
        </b-card-text>
        <canvas id="canvas"
                v-on:mousedown="startDrawing" 
                v-on:mouseup="stopDrawing" 
                v-on:mousemove="Draw" 
                v-on:touchstart="startDrawing" 
                v-on:touchend="stopDrawing"
                v-on:touchcancel="stopDrawing"
                v-on:touchmove="Draw"
                width="10" 
                height="10" 
                style="
                  border:1px solid #000000; 
                  margin: auto; 
                  padding: 0; 
                  display: block;
                  "
                >
        </canvas>
        
      </b-card>
    
      
      
    </b-container>
  </div>
  `,




    //~ <b-card body-text-variant="white" v-bind:style="{ backgroundColor: color_blue}">
        //~ <b-row>
          //~ <b-col>
            //~ <label for="lin_vel_range" style="font-size: 30px">Linear velocity: {{ lin_value }} </label>
          //~ </b-col>
          //~ <b-col cols="7">
            //~ <b-form-input id="lin_vel_range" v-model="lin_value" type="range" 
                          //~ min="0.05" max="0.7" step="0.01"></b-form-input>
          //~ </b-col>
        //~ </b-row>
        //~ <b-row>
          //~ <b-col>
            //~ <label for="ang_vel_range" style="font-size: 30px">Angular velocity: {{ ang_value }}</label>
          //~ </b-col>
          //~ <b-col cols="7">
            //~ <b-form-input id="ang_vel_range" v-model="ang_value" type="range" 
                          //~ min="0.05" max="1.0" step="0.01" size="lg"></b-form-input>
          //~ </b-col>
        //~ </b-row>
      //~ </b-card>
      
      //~ <b-row class="mb-4 mt-4" >
        //~ <b-col cols="4">
          //~ <b-button block v-on:click="flFunc" style="backgroundColor: #1e2b4e;">
            //~ <b-icon icon="arrow-up-left-circle" 
                    //~ style="width: 100%; height: 100%;"></b-icon>
          //~ </b-button>
        //~ </b-col>
        //~ <b-col cols="4">
          //~ <b-button block v-on:click="fFunc" style="backgroundColor: #1e2b4e;">
            //~ <b-icon icon="arrow-up-circle" 
                    //~ style="width: 100%; height: 100%;"></b-icon>
          //~ </b-button>
        //~ </b-col>
        //~ <b-col cols="4">
          //~ <b-button block v-on:click="frFunc" style="backgroundColor: #1e2b4e;">
            //~ <b-icon icon="arrow-up-right-circle" 
                    //~ style="width: 100%; height: 100%;"></b-icon>
          //~ </b-button>
        //~ </b-col>
      //~ </b-row >
      //~ <b-row class="mb-4">
        //~ <b-col cols="4">
          //~ <b-button block v-on:click="lFunc" style="backgroundColor: #1e2b4e;">
            //~ <b-icon icon="arrow-left-circle" 
                    //~ style="width: 100%; height: 100%;"></b-icon>
          //~ </b-button>
        //~ </b-col>
        //~ <b-col cols="4">
          //~ <b-button block v-on:click="sFunc" style="backgroundColor: #1e2b4e;">
            //~ <b-icon icon="stop-circle" 
                    //~ style="width: 100%; height: 100%;"></b-icon>
          //~ </b-button>
        //~ </b-col>
        //~ <b-col cols="4">
          //~ <b-button block v-on:click="rFunc" style="backgroundColor: #1e2b4e;">
            //~ <b-icon icon="arrow-right-circle" 
                    //~ style="width: 100%; height: 100%;"></b-icon>
          //~ </b-button>
        //~ </b-col>
      //~ </b-row>
      //~ <b-row class="mb-4">
        //~ <b-col cols="4">
          //~ <b-button block v-on:click="blFunc" style="backgroundColor: #1e2b4e;">
            //~ <b-icon icon="arrow-down-left-circle" 
                    //~ style="width: 100%; height: 100%;"></b-icon>
          //~ </b-button>
        //~ </b-col>
        //~ <b-col cols="4">
          //~ <b-button block v-on:click="bFunc" style="backgroundColor: #1e2b4e;">
            //~ <b-icon icon="arrow-down-circle" 
                    //~ style="width: 100%; height: 100%;"></b-icon>
          //~ </b-button>
        //~ </b-col>
        //~ <b-col cols="4">
          //~ <b-button block v-on:click="brFunc" style="backgroundColor: #1e2b4e;">
            //~ <b-icon icon="arrow-down-right-circle" 
                    //~ style="width: 100%; height: 100%;"></b-icon>
          //~ </b-button>
        //~ </b-col>
      //~ </b-row>
  
  data(){
    return {
      color_blue: '#1e2b4e',
      color_yellow: '#fab001',
      lin_value: 0.5,
      ang_value: 0.5,
      velocities: {
        lin: 0.0,
        ang: 0.0
      },
      
      x_coordinate: 0.0,
      y_coordinate: 0.0,
      speed: 0.0,
      angle: 0.0,
      canvas: null,
      ctx: null,
      x_coord_elem: null,
      y_coord_elem: null,
      speed_elem: null,
      angle_elem: null,
      width: 0.0,
      height: 0.0,
      radius: 0.0,
      x_orig: 0.0,
      y_orig: 0.0,
      coord: {
        x: 0,
        y: 0 
      },
      paint: false,
      mouse_x: 0.0,
      mouse_y: 0.0,
      current_radius: 0.0,
      angle_in_degrees: 0.0,
      x: 0.0,
      y: 0.0,
      x_relative: Math.round(this.x - this.x_orig),
      y_relative: Math.round(this.y - this.y_orig)
    };
  },
  
  mounted: function(){
    this.x_coordinate = 0;
    this.y_coordinate = 0;
    this.speed = 0;
    this.angle = 0;
    
    window.addEventListener('load', () => {
      
      this.canvas = document.getElementById('canvas');
      this.ctx = canvas.getContext('2d');
      this.resize();
      
      document.addEventListener('mousedown', this.startDrawing);
      document.addEventListener('mouseup', this.stopDrawing);
      document.addEventListener('mousemove', this.Draw);

      document.addEventListener('touchstart', this.startDrawing);
      document.addEventListener('touchend', this.stopDrawing);
      document.addEventListener('touchcancel', this.stopDrawing);
      document.addEventListener('touchmove', this.Draw);

      window.addEventListener('resize', this.resize());
      
      //~ document.getElementById("x_coordinate").innerText = 0;
      //~ document.getElementById("y_coordinate").innerText = 0;
      //~ document.getElementById("speed").innerText = 0;
      //~ document.getElementById("angle").innerText = 0;
    });
    
    
  },
  
  methods: {
    
    send: function(x,y,speed,angle){
      this.x_coordinate = x;
      this.y_coordinate = y;
      this.speed = speed;
      this.angle = angle;
      
    },
    
    resize: function(){
      this.width = window.innerWidth;
      this.radius = 200;
      this.height = this.radius * 6.5;
      this.ctx.canvas.width = this.width;
      this.ctx.canvas.height = this.height;
      this.background();
      this.joystick(this.width / 2, this.height / 3);
    },

    background: function(){
      this.x_orig = this.width / 2;
      this.y_orig = this.height / 3;
      this.ctx.beginPath();
      this.ctx.arc(this.x_orig, this.y_orig, this.radius + 20, 0, Math.PI * 2, true);
      this.ctx.fillStyle = '#ECE5E5';
      this.ctx.fill();
    },

    joystick: function(widht, height){
      this.ctx.beginPath();
      this.ctx.arc(this.width, this.height, this.radius, 0, Math.PI * 2, true);
      this.ctx.fillStyle = '#F08080';
      this.ctx.fill();
      this.ctx.strokeStyle = '#F6ABAB';
      this.ctx.lineWidth = 8;
      this.ctx.stroke();
    },

    getPosition: function(event){
      this.mouse_x = event.clientX || event.touches[0].clientX;
      this.mouse_y = event.clientY || event.touches[0].clientY;
      this.coord.x = this.mouse_x - this.canvas.offsetLeft;
      this.coord.y = this.mouse_y - this.canvas.offsetTop;
    },
    
    is_it_in_the_circle: function(){
      this.current_radius = Math.sqrt(
        Math.pow(this.coord.x - this.x_orig, 2) + Math.pow(this.coord.y - this.y_orig, 2));
      if (this.radius >= this.current_radius) return true
      else return false
    },

    startDrawing: function(event){
      this.paint = true;
      this.getPosition(event);
      if (this.is_it_in_the_circle()) {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        this.background();
        joystick(this.coord.x, this.coord.y);
        Draw();
      }
    },

    stopDrawing: function(){
      this.paint = false;
      this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.background();
      this.joystick(this.width / 2, this.height / 3);
      
      this.x_coordinate = 0;
      this.y_coordinate = 0;
      this.speed = 0;
      this.angle = 0;
      
      //~ document.getElementById("x_coordinate").innerText = 0;
      //~ document.getElementById("y_coordinate").innerText = 0;
      //~ document.getElementById("speed").innerText = 0;
      //~ document.getElementById("angle").innerText = 0;
    },

    Draw: function(event){
      if (this.paint) {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        this.background();
        this.angle = Math.atan2((this.coord.y - this.y_orig), (this.coord.x - this.x_orig));

        if (Math.sign(this.angle) == -1) {
          this.angle_in_degrees = Math.round(-(this.angle) * 180 / Math.PI);
        }
        else {
          this.angle_in_degrees = Math.round( 360 - this.angle * 180 / Math.PI);
        }

        if (this.is_it_in_the_circle()) {
          this.joystick(this.coord.x, this.coord.y);
          this.x = this.coord.x;
          this.y = this.coord.y;
        }
        else {
          this.x = this.radius * Math.cos(this.angle) + this.x_orig;
          this.y = this.radius * Math.sin(this.angle) + this.y_orig;
          this.joystick(this.x, this.y);
        }
        
        this.getPosition(event);

        this.speed =  Math.round(100 * Math.sqrt(Math.pow(this.x - this.x_orig, 2) + 
                      Math.pow(this.y - this.y_orig, 2)) / this.radius);

        this.x_relative = Math.round(this.x - this.x_origin);
        this.y_relative = Math.round(this.y - this.y_origin);

        this.x_coordinate = this.x_relative;
        this.y_coordinate = this.y_relative;

        this.x_coord_elem = document.getElementById("x_coordinate").innerText;
        this.y_coord_elem = document.getElementById("y_coordinate").innerText;
        this.speed_elem = document.getElementById("speed").innerText;
        this.angle_elem = document.getElementById("angle").innerText;

        x_coord_elem = this.x_relative;
        y_coord_elem = this.y_relative ;
        speed_elem = this.speed;
        angle_elem = this.angle_in_degrees;

        this.send(this.x_relative, this.y_relative, this.speed, this.angle_in_degrees);
      }
    },
    
    //~ updateHTML: function(elmId, value){
      //~ var elem = document.getElementById(elmId);
      //~ if(typeof elem !== 'undefined' && elem !== null) {
        //~ elem.innerHTML = value;
        //~ }

    //~ },
    
    flFunc: function(){
      this.velocities.lin = this.lin_value;
      this.velocities.ang = this.ang_value;
      socket.emit('twist_message', JSON.stringify(this.velocities));
    },
    
    fFunc: function(){
      this.velocities.lin = this.lin_value;
      this.velocities.ang = 0.0;
      socket.emit('twist_message', JSON.stringify(this.velocities));
    },
    
    frFunc: function(){
      this.velocities.lin = this.lin_value;
      this.velocities.ang = -this.ang_value;
      socket.emit('twist_message', JSON.stringify(this.velocities));
    },
    
    lFunc: function(){
      this.velocities.ang = this.ang_value;
      this.velocities.lin = 0.0;
      socket.emit('twist_message', JSON.stringify(this.velocities));
    },
    
    sFunc: function(){
      this.velocities.lin = 0.0;
      this.velocities.ang = 0.0;
      socket.emit('twist_message', JSON.stringify(this.velocities));
    },
    
    rFunc: function(){
      this.velocities.ang = -this.ang_value;
      this.velocities.lin = 0.0;
      socket.emit('twist_message', JSON.stringify(this.velocities));
    },
    
    blFunc: function(){
      this.velocities.lin = -this.lin_value;
      this.velocities.ang = this.ang_value;
      socket.emit('twist_message', JSON.stringify(this.velocities));
    },
    
    bFunc: function(){
      this.velocities.lin = -this.lin_value;
      this.velocities.ang = 0.0;
      socket.emit('twist_message', JSON.stringify(this.velocities));
    },
    
    brFunc: function(){
      this.velocities.lin = -this.lin_value;
      this.velocities.ang = -this.ang_value;
      socket.emit('twist_message', JSON.stringify(this.velocities));
    }
  }
});



