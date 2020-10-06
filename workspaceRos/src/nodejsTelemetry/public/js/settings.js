$("#navSettings").addClass("active")

var settings = [];

var updateSettings = function(set) {
    $(".settings").empty();
    set.forEach(s => {
        $(".settings").append(
            '<li class="list-group-item container">\
            <div class="row">\
                <div class="col-md-9">\
                    <p class="text-primary">' + s.variable + '</p>\
                    <p class="text-secondary">' + s.description + '</p>\
                </div>\
                <div class="col-md-3">\
                    <input class="text inputSetting form-control my-4" id="' + s.variable + '" value="' + s.value + '">\
                </div>\
            </div>\
        </li>'
        );

        $('#' + s.variable).on('input', function() {
            s.value = $('#' + s.variable).val(); //sanitize
        });
    });
}

socket.on("yourSettings", function (data) {
    settings = data;
    updateSettings(data);
})

socket.emit("gimmeSettings");

socket.on("settings", function(data) {
    console.log("received");
    $('#newSettingsModal').modal('show');
    $("#updateSocketSettings").click(function (event) {
        $('#newSettingsModal').modal('hide');
        settings = data;
        updateSettings(data);
    });
});

$("#submitSettings").click(function (e) { 
    socket.emit("newSettings", settings);
});