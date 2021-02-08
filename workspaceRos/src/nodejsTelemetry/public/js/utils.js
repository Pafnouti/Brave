$("#routing").click(function(event) {
    console.log("Ding");
    if ($('#routing').is(":checked"))
    {
        socket.emit('Routing', true);
    } else {
        socket.emit('Routing', false);
    }
});