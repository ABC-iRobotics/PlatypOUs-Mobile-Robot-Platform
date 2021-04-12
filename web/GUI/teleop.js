Vue.component("teleop", {
  template: `
  <div>
    <b-form>
      <b-form-input v-model="value" type="range" min="-2.0" max="2.0" step="0.1" v-on:change="twistFunc"></b-form-input>
      <div class="mt-2">Value: {{ value }}</div>
      <b-button v-on:click="twistReset">Reset</b-button>
    </b-form>
  </div>`,
  
  data(){
    return {
      value: 0.0
    };
  },
  
  methods: {
    twistFunc: function(){
      socket.emit('twist_message', this.value);
    },
    
    twistReset: function(){
      socket.emit('twist_message', 0.0);
      this.value = 0.0;
    }
  }
});
