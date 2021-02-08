

var socket = io.connect();

var sendWaypoints = function(wp) {
    socket.emit("newMission", wp);
};

var currWP = NaN;

var state = {
    x:0,
    y:0,
    heading: 20,
    SOG: 0,
    COG: 0,
    TWS: 0,
    TWA: 0
};

socket.on('state', function (newState) {
    state = newState;
});

window.onload = function() {
    $("#routing").click((function() {
        if ($('#routing').is(":checked"))
        {
            socket.emit('Routing', true)
        } else {
            socket.emit('Routing', false)
        }
    }));
}