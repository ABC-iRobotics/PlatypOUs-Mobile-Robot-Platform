

Vue.component("test-comp", {
  template: `
  <div>
    <p>{{ output }}</p>
    <input v-model="message">
    <button v-on:click="click"> Push me :) </button>
  </div>`,
  
  data(){
    return {
      output: "default output",
      message: "default message"
    };
  },
  
  methods: {
    click: function(){
      this.output = this.message;
      socket.emit('twist_message', 0.0);
    }
  }
});
