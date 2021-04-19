Vue.component("teleop", {
  template: `
  <div >
    <b-container fluid>
      <b-card body-text-variant="white" v-bind:style="{ backgroundColor: color_blue}">
        <b-row>
          <b-col>
            <label for="lin_vel_range" style="font-size: 30px">Linear velocity: {{ lin_value }} </label>
          </b-col>
          <b-col cols="7">
            <b-form-input id="lin_vel_range" v-model="lin_value" type="range" 
                          min="0.05" max="0.7" step="0.01"></b-form-input>
          </b-col>
        </b-row>
        <b-row>
          <b-col>
            <label for="ang_vel_range" style="font-size: 30px">Angular velocity: {{ ang_value }}</label>
          </b-col>
          <b-col cols="7">
            <b-form-input id="ang_vel_range" v-model="ang_value" type="range" 
                          min="0.05" max="1.0" step="0.01" size="lg"></b-form-input>
          </b-col>
        </b-row>
      </b-card>
      
      <b-row class="mb-4 mt-4" >
        <b-col cols="4">
          <b-button block v-on:click="flFunc" style="backgroundColor: #1e2b4e;">
            <b-icon icon="arrow-up-left-circle" 
                    style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col cols="4">
          <b-button block v-on:click="fFunc" style="backgroundColor: #1e2b4e;">
            <b-icon icon="arrow-up-circle" 
                    style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col cols="4">
          <b-button block v-on:click="frFunc" style="backgroundColor: #1e2b4e;">
            <b-icon icon="arrow-up-right-circle" 
                    style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
      </b-row >
      <b-row class="mb-4">
        <b-col cols="4">
          <b-button block v-on:click="lFunc" style="backgroundColor: #1e2b4e;">
            <b-icon icon="arrow-left-circle" 
                    style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col cols="4">
          <b-button block v-on:click="sFunc" style="backgroundColor: #1e2b4e;">
            <b-icon icon="stop-circle" 
                    style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col cols="4">
          <b-button block v-on:click="rFunc" style="backgroundColor: #1e2b4e;">
            <b-icon icon="arrow-right-circle" 
                    style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
      </b-row>
      <b-row class="mb-4">
        <b-col cols="4">
          <b-button block v-on:click="blFunc" style="backgroundColor: #1e2b4e;">
            <b-icon icon="arrow-down-left-circle" 
                    style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col cols="4">
          <b-button block v-on:click="bFunc" style="backgroundColor: #1e2b4e;">
            <b-icon icon="arrow-down-circle" 
                    style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col cols="4">
          <b-button block v-on:click="brFunc" style="backgroundColor: #1e2b4e;">
            <b-icon icon="arrow-down-right-circle" 
                    style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
      </b-row>
    </b-container>
  </div>
  
  `,
  
  //~ <b-container fluid>
      //~ <b-card>
        //~ <b-card-text>
          //~ X: {{ x_coordinate }}
          //~ Y: {{ y_coordinate }}
          //~ Speed: {{ speed }} %
          //~ Angle: {{ angle }}
        //~ </b-card-text>
        //~ <canvas id="canvas" name="game"></canvas>
      //~ </b-card>
    //~ </b-container>
  //~ </div>
    
  
  data(){
    return {
      color_blue: '#1e2b4e',
      color_yellow: '#fab001',
      lin_value: 0.5,
      ang_value: 0.5,
      velocities: {
        lin: 0.0,
        ang: 0.0
      }
      
      //~ x_coordinate: 0.0,
      //~ y_coordinate: 0.0,
      //~ speed: 0.0,
      //~ angle: 0.0
      
    };
  },
  
  methods: {
        
    
    
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


//~ sendFunc: function(x,y,speed,angle){
      //~ this.x_coordinate = x;
      //~ this.y_coordinate = y;
      //~ this.speed = speed;
      //~ this.angle = angle;
      //~ socket.emit();
    //~ }
