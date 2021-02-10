$("#navRoslog").addClass("active")

var logs = [];

var updateLogs = function() {
    $(".logs").empty();
    logs.forEach(element => {
        $(".logs").append(
        '<li class="list-group-item container">\
        <div class="row">\
            <div class="col-md-9">\
                <p class="text-primary">' + element.name + '</p>\
                <p class="text-secondary">' + element.msg + '</p>\
            </div>\
        </div>\
    </li>'
    );
    });
    
}

socket.on("rosInfo", function (data) {
    console.log(data);
    logs.unshift(data);
    logs.length = 10;
    updateLogs();
})

socket.emit("getRosInfo");