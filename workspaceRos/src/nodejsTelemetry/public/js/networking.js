

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
    TWA: 0,
    lat0: 48.431775,
    lon0: -4.615529
};

socket.on('state', function (newState) {
    state = newState;
    lat0 = newState.lat0;
    lon0 = newState.lon0;
});

window.onload = function() {
    $("#routing").on('toggle', (function() {
        if ($('#routing').is(":checked"))
        {
            socket.emit('Routing', true)
        } else {
            socket.emit('Routing', false)
        }
    }));
}