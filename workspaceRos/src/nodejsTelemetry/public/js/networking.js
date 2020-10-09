
var socket = io.connect();

var sendWaypoints = function(wp) {
    socket.emit("newMission", wp);
};

socket.on("currentTarget", function (data) {
    $("#wpid").text(data);
});