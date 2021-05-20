Vue.component("diagnostics", {
  template: `
  <div>
    <b-container fluid>
      <b-card style="backgroundColor: #1e2b4e; color: #fab001;">
        <b-card-text>
          <p v-for="(stat, name) in robot_status">{{ name }}: {{ stat }}</p>
        </b-card-text>
      </b-card>
    </b-container>
  </div>
  `,
  
  data(){
    return {
      robot_status: new Object()
    };
  },
  
  methods: { 
    updateRobotStatus: function(data){
      this.robot_status = data;
    }
  }
});

