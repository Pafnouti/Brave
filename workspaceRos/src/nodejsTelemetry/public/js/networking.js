
var socket = io.connect();

var sendWaypoints = function(wp) {
    socket.emit("newMission", wp);
};

socket.on("currentTarget", function (data) {
    console.log(data);
    $("#wpid").text(data);
});