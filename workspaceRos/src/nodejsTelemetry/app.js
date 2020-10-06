var express = require('express');
var app = express();
var server = require('http').Server(app);
var io = require('socket.io')(server);

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
    variable:"line_Distance",
    type: "int",
    value: 5,
    description: "Allowed distance to the line"
  },
  {
    variable:"tacking_Angle",
    type: "int",
    value: 35,
    description: "Minimum allowed tacking angle"
  },
];

io.on('connection', function (socket) {
  console.log("Some one connected !")

  socket.on('newMission', function (data) {
    waypoints = data; //sanitize here ?
    console.log(waypoints);
    socket.broadcast.emit('waypoints', waypoints);
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