Vue.component("teleop", {
  template: `
  <div>
    <b-container>
      <b-row>
        <b-col>
          <label for="lin_vel_range">Linear velocity: {{ lin_value }} </label>
        </b-col>
        <b-col cols="8">
          <b-form-input id="lin_vel_range" v-model="lin_value" type="range" min="0.05" max="0.7" step="0.01"></b-form-input>
        </b-col>
      </b-row>
      <b-row>
        <b-col>
          <label for="ang_vel_range">Linear velocity: {{ ang_value }}</label>
        </b-col>
        <b-col cols="8">
          <b-form-input id="ang_vel_range" v-model="ang_value" type="range" min="0.05" max="1.0" step="0.01"></b-form-input>
        </b-col>
      </b-row>
    </b-container>
    <b-container class="bv-example-row bv-example-row-flex-cols">
      <b-row>
        <b-col>
          <b-button block size="lg" v-on:click="flFunc">
            <b-icon icon="arrow-up-left-circle"></b-icon>
          </b-button>
        </b-col>
        <b-col >
          <b-button block size="lg" v-on:click="fFunc">
            <b-icon icon="arrow-up-circle"></b-icon>
          </b-button>
        </b-col>
        <b-col >
          <b-button block size="lg" v-on:click="frFunc">
            <b-icon icon="arrow-up-right-circle"></b-icon>
          </b-button>
        </b-col>
      </b-row>
      <b-row align-v="stretch">
        <b-col>
          <b-button block size="lg" v-on:click="lFunc">
            <b-icon icon="arrow-left-circle"></b-icon>
          </b-button>
        </b-col>
        <b-col>
          <b-button block size="lg" v-on:click="sFunc">
            <b-icon icon="stop-circle"></b-icon>
          </b-button>
        </b-col>
        <b-col>
          <b-button block size="lg" v-on:click="rFunc">
            <b-icon icon="arrow-right-circle"></b-icon>
          </b-button>
        </b-col>
      </b-row>
      <b-row align-v="stretch">
        <b-col>
          <b-button block size="lg" v-on:click="blFunc">
            <b-icon icon="arrow-down-left-circle"></b-icon>
          </b-button>
        </b-col>
        <b-col>
          <b-button block size="lg" v-on:click="bFunc">
            <b-icon icon="arrow-down-circle"></b-icon>
          </b-button>
        </b-col>
        <b-col>
          <b-button block size="lg" v-on:click="brFunc">
            <b-icon icon="arrow-down-right-circle"></b-icon>
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
