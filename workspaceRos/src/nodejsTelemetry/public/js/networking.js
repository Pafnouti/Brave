
var socket = io.connect();

var sendWaypoints = function(wp) {
    socket.emit("newMission", wp);
};