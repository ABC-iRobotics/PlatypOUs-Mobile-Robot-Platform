Vue.component("teleop", {
  template: `
  <div >
    <b-container fluid>
      <b-row>
        <b-col>
          <label for="lin_vel_range" style="font-size: 30px">Linear velocity: {{ lin_value }} </label>
        </b-col>
        <b-col cols="8">
          <b-form-input id="lin_vel_range" v-model="lin_value" type="range" 
                        min="0.05" max="0.7" step="0.01"></b-form-input>
        </b-col>
      </b-row>
      <b-row>
        <b-col>
          <label for="ang_vel_range" style="font-size: 30px">Linear velocity: {{ ang_value }}</label>
        </b-col>
        <b-col cols="8">
          <b-form-input id="ang_vel_range" v-model="ang_value" type="range" 
                        min="0.05" max="1.0" step="0.01" size="lg"></b-form-input>
        </b-col>
      </b-row>
    </b-container>
    <b-container >
      <b-row class="mb-4" >
        <b-col>
          <b-button v-on:click="flFunc" style="width: 100%; height: 100%;">
            <b-icon icon="arrow-up-left-circle" style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col >
          <b-button v-on:click="fFunc" style="width: 100%; height: 100%;">
            <b-icon icon="arrow-up-circle" style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col >
          <b-button  v-on:click="frFunc" style="width: 100%; height: 100%;">
            <b-icon icon="arrow-up-right-circle" style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
      </b-row >
      <b-row class="mb-4">
        <b-col>
          <b-button  v-on:click="lFunc" style="width: 100%; height: 100%;">
            <b-icon icon="arrow-left-circle" style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col>
          <b-button  v-on:click="sFunc" style="width: 100%; height: 100%;">
            <b-icon icon="stop-circle" style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col>
          <b-button  v-on:click="rFunc" style="width: 100%; height: 100%;">
            <b-icon icon="arrow-right-circle" style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
      </b-row>
      <b-row class="mb-4">
        <b-col>
          <b-button  v-on:click="blFunc" style="width: 100%; height: 100%;">
            <b-icon icon="arrow-down-left-circle" style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col>
          <b-button  v-on:click="bFunc" style="width: 100%; height: 100%;">
            <b-icon icon="arrow-down-circle" style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
        <b-col>
          <b-button v-on:click="brFunc" style="width: 100%; height: 100%;">
            <b-icon icon="arrow-down-right-circle" style="width: 100%; height: 100%;"></b-icon>
          </b-button>
        </b-col>
      </b-row>
    </b-container>
  </div>`,
  
  data(){
    return {
      lin_value: 0.5,
      ang_value: 0.5,
      velocities: {
        lin: 0.0,
        ang: 0.0
      }
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
