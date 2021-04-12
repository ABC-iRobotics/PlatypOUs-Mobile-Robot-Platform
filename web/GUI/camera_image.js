Vue.component("camera-image", {
  template: `
  <div>
    <label for="camera_live" 
            style="font-size: x-large; text-indent: 30pt; line-height: 50px;">Live camera footage:</label>
    <canvas id="camera_image" width="640" height="360" 
              style="border:1px solid #000000; margin: auto; padding: 0; display: block;"></canvas>
  </div>`,
  
  data(){
    return {
      img: new Image(),
      ctx: null
    };
  },
  
  methods: {
    renderImage: function(image_data){
      this.img.src = 'data:image/jpeg;base64, ' + image_data;
      this.ctx.drawImage(this.img, 0, 0);
    }
  },
  
  mounted: function(){        
    this.ctx = document.getElementById('camera_image').getContext('2d');
  },
});
