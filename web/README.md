# PlatypOUs Web GUI

Web server and client implementation of the PlatypOUs differential drive mobile robot platform.

## Developer documentation

### Requirements
- npm: 5.8.0
- Node.js: v10.24.0
- npm packages:
    - arraybuffer-to-string v1.0.2
    - express: v4.17.1
    - rosnodejs: v3.0.2
    - socket.io: v4.0.1

### Server side

The *server.js* file is responsible for starting the web server and using the packages correctly.

Through Socket.IO, the code establishes the connection and with it's help we can listen to certain events and then publish the message. This can be done the following way:

```js=
io.on('connection', (socket) => {
  socket.on('message', (msg) => {
    pub.publish(msg);
  });
});
```

After this, with the help of *rosnodejs* this can be advertised to ROS topics:

```js=
const nh = rosnodejs.nh;
.
.
.
pub = nh.advertise('/topic_name', "topicType");
```

On the other hand, listening to a ROS topic and emitting it's message through Socket.IO looks like this:
```js=
nh.subscribe("/topic_name", "topicType", (msg) => {
    io.emit("event-name", msg);
});
```

The HTTP server to port *3000* is created with the next code:
```js=
http.listen(3000, () => {
  console.log('listening on *:3000');
});
```

### Client side

#### Vue components

The client side is based on Vue.js and Vue components. The main Vue instance is *main_vue.js* and the components are *teleop.js*, *navigation.js* and *diagnostics.js*. These have the same kind of structure.

This consists of *data*, *mounted* and *methods* parts, where the *data* stores all the used parameters and datas, the *mounted* helps defining the functions which should load with the web page and *methods* collects the functions and usabilities of the program.

Every component has a name which is used from outside of the files, this is the code for it:

```js=
Vue.component("component_name", {...});
```

Inside the components there is the *template* field which contains the view of the code, in HTML format.

#### Communication

The file *socket_io.js* handles the two-way communication on the client side. An event listener can be added the following way:

```js=
socket.on('event-name', function(data) {
  App.$refs.ref_name.functionName(data);
});
```

#### Visualization

The *index.html* is responsible for visualizing the web page. All elements are written with the help of BootstrapVue HTML tags. The tabs on the user interface are made the following way:

```html=
<b-tabs pills card  justified style="font-size: 30px;">
  <b-tab title="Teleoperation" active :title-link-class="'tab-title-class'">
    <teleop ref="teleop_ref"></teleop>
  </b-tab>
  <b-tab title="Navigation" :title-link-class="'tab-title-class'">
    <navigation ref="navigation_ref"></navigation>
  </b-tab>
  <b-tab title="Diagnostics" :title-link-class="'tab-title-class'">
    <diagnostics ref="diagnostics_ref"></diagnostics>
  </b-tab>
</b-tabs>
```

By adding *<b-tab>* elements any number of tabs can be made. The *ref* parameters are used by *socket_io.js* for finding the Vue components' methods. We can see that the *<teleop>*, *<navigation>*, *<diagnostics>* tags represent the components by their names.
