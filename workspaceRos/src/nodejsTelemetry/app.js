var express = require('express');
var app = express();
var server = require('http').Server(app);
var io = require('socket.io')(server);

const rosnodejs = require('rosnodejs');

var path = require('path');
var routes = require('./routes/index');
var bodyParser = require('body-parser');

app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'pug');
app.use(bodyParser.urlencoded({ extended: true }));

app.use('/', routes);
app.use(express.static('public'));

module.exports = server;

var waypoints = [
  {
    latlong: [48.370954, -4.480665],
    id: "id0"
  },
  {
    latlong: [48.380, -4.480665],
    id: "id1"
  },
  {
    latlong: [48.370954, -4.4850],
    id: "id2"
  },
  {
    latlong: [48.370954, -4.475],
    id: "id3"
  }
];
var settings = [
  {
    variable: "line_Distance",
    type: "int",
    value: 5,
    description: "Allowed distance to the line"
  },
  {
    variable: "tacking_Angle",
    type: "int",
    value: 35,
    description: "Minimum allowed tacking angle"
  },
];

var std_msgs;
var wp_pub;


std_msgs = rosnodejs.require('std_msgs').msg;
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;

// Register node with ROS master
rosnodejs.initNode('telemetry_node')
  .then((rosNode) => {
    // Create ROS subscriber on the 'chatter' topic expecting String messages
    let subState = rosNode.subscribe('/State', geometry_msgs.Pose2D,
      (data) => { // define callback execution
        state.x = data.x;
        state.y = data.y;
        state.heading = data.theta;
        socket.broadcast.emit('state', state);
      }
    );
    let subID = rosNode.subscribe('/Current_Target', std_msgs.Int32,
      (data) => { // define callback execution
        console.log(data);
        socket.broadcast.emit('currentTarget', data);
      }
    );
    wp_pub = rosNode.advertise("/Waypoints", std_msgs.Float64MultiArray)
  });


io.on('connection', function (socket) {
  console.log("Some one connected !")

  socket.on('newMission', function (data) {
    waypoints = data; //sanitize here ?
    console.log(waypoints);
    socket.broadcast.emit('waypoints', waypoints);

    const msg = new std_msgs.Float64MultiArray();

    dt = []
    waypoints.forEach(wp => {
      dt.push(wp.latlong[0]);
      dt.push(wp.latlong[1]);
    });

    msg.data = Float64Array.from(dt);
    wp_pub.publish(msg);
  });

  socket.on('gimmeWP', function (data) {
    socket.emit('yourWP', waypoints);
  });

  socket.on('gimmeSettings', function (data) {
    socket.emit('yourSettings', settings);
  });

  socket.on('newSettings', function (data) {
    settings = data; //sanitize here ?
    console.log(data);
    socket.broadcast.emit('settings', settings);
  });

});