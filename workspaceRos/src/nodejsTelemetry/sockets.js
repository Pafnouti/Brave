var handle = function (socket) {
    console.log("Some one connected !")

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
        console.log(data);
        socket.broadcast.emit('settings', settings);
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

    socket.on("routingTarget", function(data){
        tgt_msg.x = data.lat;
        tgt_msg.y = data.lon;
        routing_tgt_pub.publish(tgt_msg)
    });
    setInterval(function () {
        socket.emit('state', state);
        socket.emit('currentTarget', currWP);
        if (newWps) {
        socket.emit('staticWP', waypoints);
        newWps = false;
        }
        if (newPoly) {
        socket.emit('newPolys', polys)
        newPoly = false;
        }
        if (newLogs) {
        logs.forEach(element => {
            socket.emit('rosInfo', element)
        });
        logs = [];
        newLogs = false;
        }
        if (newCargo) {
        cargos.forEach(element => {
            socket.emit('cargos', element)
        });
        cargos = [];
        newCargo = false;
        }
    }, 1000);

};