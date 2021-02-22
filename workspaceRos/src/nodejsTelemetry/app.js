var express = require('express');
var app = express();
var server = require('http').Server(app);
var io = require('socket.io')(server);

const rosnodejs = require('rosnodejs');

var path = require('path');
var routes = require('./routes/index');
var bodyParser = require('body-parser');
const { stat } = require('fs');

app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'pug');
app.use(bodyParser.urlencoded({ extended: true }));

app.use('/', routes);
app.use(express.static('public'));

module.exports = server;

var waypoints = [
  {
    latlong: [48.375, -4.480665],
    id: "id0"
  },
  {
    latlong: [48.390, -4.480665],
    id: "id1"
  },
  {
    latlong: [48.350, -4.4850],
    id: "id2"
  },
  {
    latlong: [48.360, -4.475],
    id: "id3"
  }
];
var settings = [
  {
    variable: "router_child_number",
    type: "int",
    value: 40,
    description: "Number of points children of each point"
  },
  {
    variable: "router_angular_def",
    type: "int",
    value: 200,
    description: "Number of angular sectors"
  },
  {
    variable: "router_iso_nb",
    type: "int",
    value: 10,
    description: "Number of isochrons (= number of waypoints)"
  },
];
var currWP = 0;
var newWps = true;
var newPoly = true;
var newLogs = false;
var newCargo = true;
var polys = [];
var cargos = [];
var logs = [];
var allLogs = [];

var wp_pub;
var routing_pub;
var routing_tgt_pub;
var polys_pub;

const std_msgs = rosnodejs.require('std_msgs').msg;
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;
const rosgraph_ms = rosnodejs.require('rosgraph_msgs').msg;

var state = {
  x:0,
  y:0,
  heading: 0,
  SOG: 0,
  COG: 0,
  TWA: 0,
  lat0: 48.431775,
  lon0: -4.615529
};

var cartToWGS84 = function(x, y) {
  EARTH_RADIUS = 6371000.
  EPSILON = 0.00000000001
  lat = y*180./pi/EARTH_RADIUS+state.lat0
  lon = abs(lat-90.) < EPSILON || abs(lat+90.) < EPSILON ? 0 : (x/EARTH_RADIUS)*(180./pi)/cos((pi/180.)*(lat))+state.lon0
  
  return [lat, lon]
}

// Register node with ROS master
rosnodejs.initNode('telemetry_node')
  .then((rosNode) => {
    rosnodejs.loadAllPackages().then( (e) => {
      controller_msg = rosnodejs.require('controller').msg;
  
      // Create ROS subscriber on the 'chatter' topic expecting String messages
      let subState = rosNode.subscribe('/State', geometry_msgs.Pose2D,
        (data) => { // define callback execution
          state.x = data.x;
          state.y = data.y;
          state.heading = data.theta;
        }
      );
      let subID = rosNode.subscribe('/Current_Target', std_msgs.Int32,
        (data) => { // define callback execution
          currWP = data;
        }
      );
  
      let subGPS = rosNode.subscribe('/ublox/GPRMC', controller_msg.Gps, (data) => {
        state.SOG = data.boat_speed;
        state.COG = data.heading;
      });
      let subWind = rosNode.subscribe('/ublox/WIMDA', controller_msg.Meteo, (data) => {
        state.TWS = data.true_wind_speed;
        state.TWA = data.wind_direction;
      });
  
      let subWP = rosNode.subscribe("/Waypoints", std_msgs.Float64MultiArray, (data) => {
        latlong = data.data;
        wps = []
        for (let index = 0; index < latlong.length; index+=2) {
          wps.push({
            latlong:[latlong[index], latlong[index + 1]],
            id:index/2
          });
        }
        console.log("Waypoints received on /Waypoints topic.")
        waypoints = wps;
        newWps = true;
      });
  
      let subRosOut = rosNode.subscribe("/rosout", rosgraph_ms.Log, (data) => {
        logs.push(data);
        allLogs.push(data);
        newLogs = true;
      });
  
      // let subPolys = rosNode.subscribe("/Poly", controller_msg.UniquePolygon, (data) => {
      //   ll = []
      //   data.poly.points.forEach(element => {
      //     ll.push(cartToWGS84(element.x, element.y));
      //   });
      //   found = false
      //   for (let index = 0; index < polys.length; index++) {
      //     const element = polys[index];
      //     if (element.id==data.id) {
      //       element = ll
      //       found = true
      //     }
      //   }
      //   if (!found) {
      //     polys.push(ll)
      //   }
      //   newPoly = true
      // });
  
      let subCargos = rosNode.subscribe("/posNavire", geometry_msgs.Pose2D, (data) => {
        cargos = [data];
        newCargo = true;
      });
  
      wp_pub = rosNode.advertise("/Waypoints", std_msgs.Float64MultiArray);
      routing_pub = rosNode.advertise("/Routing", std_msgs.Bool);
      routing_tgt_pub = rosNode.advertise("/Target", geometry_msgs.Pose2D);
      console.log(controller_msg.UniquePolygonArray);
      console.log(controller_msg.UniquePolygon);
      polys_pub = rosNode.advertise("/Polys", controller_msg.UniquePolygonArray);
    }
    );
  });


io.on('connection', function (socket) {
  console.log("Some one connected !")
  newWps = true;
  newPoly = true;
  newLogs = false;
  newCargo = true;
  const wps_msg = new std_msgs.Float64MultiArray();
  socket.on('newMission', function (data) {
    waypoints = data; //sanitize here ?
    console.log(waypoints);
    //socket.emit('staticWP', waypoints);


    dt = []
    waypoints.forEach(wp => {
      dt.push(wp.latlong[0]);
      dt.push(wp.latlong[1]);
    });

    wps_msg.data = Float64Array.from(dt);
    wp_pub.publish(wps_msg);
  });

  socket.on('getStaticWP', function (data) {
    socket.emit('staticWP', waypoints);
  });

  socket.on('getRosInfo', function (data) {
    allLogs.forEach(element => {
      socket.emit('rosInfo', element);
    });
  });

  socket.on('gimmeSettings', function (data) {
    socket.emit('yourSettings', settings);
  });

  socket.on('newSettings', function (data) {
    settings = data; //sanitize here ?
    settings.forEach(element => {
      rosnodejs.setParam(element.type, element.value);
    });
    console.log(data);
    socket.broadcast.emit('settings', settings);
    newParam = true;
  });

  var tgt_msg = new geometry_msgs.Pose2D();
  tgt_msg.x = 48.4305;
  tgt_msg.y = -4.6127;

  socket.on('Routing', function(data) {
    var msg = new std_msgs.Bool();
    msg.data = data;
    routing_tgt_pub.publish(tgt_msg);
    routing_pub.publish(msg);
    console.log(data);
    /*if(data) {
    } else {
      wp_pub.publish(msg);
    }*/
  });
  
  socket.on('userPolys', function(data) {
    var msgList = new controller_msg.UniquePolygonArray();
    var list = [];
    polys = data;
    data.forEach(receivedPolygon => {    
      var msg = new controller_msg.UniquePolygon();
      var p = new geometry_msgs.Polygon();
      var obs = new std_msgs.Bool();

      console.log(receivedPolygon);
  
      var ps = [];
      receivedPolygon.latlng.forEach(element => {
        var pt = new geometry_msgs.Point32();
        pt.x = element[0];
        pt.y = element[1];
        pt.z = 0;
        ps.push(pt);
      });
      
      p.points = ps;
      msg.poly = p;

      obs.data = receivedPolygon.isObstacle;
      msg.isObstacle = obs;

      console.log(msg);
      list.push(msg);
    });

    msgList.data = list;
    polys_pub.publish(msgList);
    /*if(data) {
    } else {
      wp_pub.publish(msg);
    }*/
  });

  socket.on("routingTarget", function(data){
    tgt_msg.x = data.lat;
    tgt_msg.y = data.lon;
    routing_tgt_pub.publish(tgt_msg)
  });
  socket.on("gotWP", function(data){
    newWps = false;
    console.log("Comfirmed wp reception");
  });
  socket.on("gotPolys", function(data){
    newPoly = false;
    console.log("Comfirmed polys reception");
  });
  setInterval(function () {
    socket.broadcast.emit('state', state);
    socket.broadcast.emit('currentTarget', currWP);
    if (newWps) {
      socket.emit('staticWP', waypoints);
    }
    if (newPoly) {
      socket.emit('newPolys', polys);
    }
    if (newLogs) {
      logs.forEach(element => {
        socket.broadcast.emit('rosInfo', element)
      });
      logs = [];
      newLogs = false;
    }
    if (newCargo) {
      cargos.forEach(element => {
        socket.broadcast.emit('cargos', element)
      });
      cargos = [];
      newCargo = false;
    }
  }, 1000);
  
});