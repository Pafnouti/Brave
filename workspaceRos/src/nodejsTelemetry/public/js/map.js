$("#navMission").addClass("active")

var wayPointsList = [];

/// Socket.io
socket.on("yourWP", function(data) {
    currID = data.length;
    updateWPList(data);
    updatePath();
});

socket.emit("gimmeWP");

socket.on('waypoints', function(wp){
    $('#newWPModal').modal('show');
    $("#updateSocketWP").click(function (event) {
        $('#newWPModal').modal('hide');
        updateWPList(wp);
        updatePath();
    });
});

///Leaflet.js
var baseMap = L.tileLayer('http://a.tile.openstreetmap.fr/hot/{z}/{x}/{y}.png', {
    attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
});

var buoys = L.tileLayer('http://t1.openseamap.org/seamark/{z}/{x}/{y}.png', {
    attribution: 'Map data: &copy; <a href="http://www.openseamap.org">OpenSeaMap</a> contributors'
});

var map = L.map('mapdiv', {
    layers: [baseMap]
}).setView([48.370954, -4.480665], 13);


baseLayers = {
    "Fond de carte": baseMap
}
overlays = {
    "Bouées": buoys
}
L.control.layers(baseLayers, overlays).addTo(map);
map.attributionControl.setPrefix(false);

var boatIcon = new L.Icon.Default();
boatIcon.options.shadowSize = [0, 0];
boatIcon.options.iconUrl = "boat_icon.png";
var size = .6;
boatIcon.options.iconAnchor = [30 * size, 60 * size];
boatIcon.options.iconSize = [61 * size, 100 * size];

var myMovingMarker = new L.marker([48.370954, -4.480665], {
    icon: boatIcon
}).addTo(map);

var addingWp = false;
map.on('click', function (e) {
    $('.wpItem.active').removeClass("active");
    if (addingWp) {
        var id = "id" + currID;
        var latlong = e.latlng;

        wayPointsList.push({
            "latlong": [latlong.lat, latlong.lng],
            "id": id
        });

        updateWPList(wayPointsList);
        updatePath();

        currID++;
        $(".deleteWP").click(function (event) {
            $(this).closest("li").remove();
        });
    }
});

var updatePath = function () {
    var latlongs = [];
    wayPointsList.forEach(element => {
        latlongs.push(element.latlong);
    });
    polyline.setLatLngs(latlongs);
    decorator.setPaths(polyline);
}

var currID = 5;

var deleteWPf = function (event) {
    var id = $(this).closest("li").attr("id");
    var i = 0;
    var k = 0;
    wayPointsList.forEach(element => {
        if (id == element.id) {
            map.removeLayer(element.marker);
            k = i;
        }
        i++;
    });
    wayPointsList.splice(k, 1);
    $(this).closest("li").remove();
    updatePath();
};

///Bootstrap
var el = document.getElementById('wayPointsList');

var swapArrayElements = function (arr, indexA, indexB) {
    var temp = arr[indexA];
    arr[indexA] = arr[indexB];
    arr[indexB] = temp;
};

var sortable = Sortable.create(el, {
    onEnd: function (evt) {
        var itemEl = evt.item;  // dragged HTMLElement
        console.log(itemEl.id, evt.oldIndex, evt.newIndex);
        swapArrayElements(wayPointsList, evt.oldIndex, evt.newIndex);
        updatePath();
    }
});

///JQuery
$('.col-md-3').click(function() {
    $('.wpItem.active').removeClass("active");
});

$(".deleteWP").click(deleteWPf);

$('#addWPModal').collapse({
    toggle: false
});

$('#addWPCancel').click(function (event) {
    addingWp = false;
    console.log("cc");
    $('#addWPModal').collapse('hide');    
});

$(".addWP").click(function (event) {
    $('#addWPModal').collapse('show');
    console.log("cc1");
    addingWp = true;
});

$(".submitWP").click(function (event) {
    var wp = []
    var id = 0;

    wayPointsList.forEach(element => {
        wp.push({
            "latlong": element.latlong,
            "id": "id" + id
        });
        id++;
    });

    sendWaypoints(wp);
});

var posShow = function (position) {
    return (Math.trunc(10000 * position.lat) / 10000 + ' : ' + Math.trunc(10000 * position.lng) / 10000)
}

var updateWPList = function (wps) {
    //wps must contain an id and a latlong array at the bare minimum
    wayPointsList.forEach(element => {
        if (element.marker) {
            map.removeLayer(element.marker);
        }
    });

    wayPointsList = [];
    wps.forEach(wp => {
        if (!wp.marker) {
            var marker = new L.marker(wp.latlong, {
                draggable: 'true'
            });
            marker.on('dragend', function (event) {
                var position = marker.getLatLng();
                marker.setLatLng(position, {
                    draggable: 'true'
                }).update();
                wp.latlong = [position.lat, position.lng];

                document.getElementById(wp.id).childNodes[0].nodeValue = posShow(position);

                updatePath();
            });
            wp.marker = marker;

            marker.on('click', function (event) {
                $('.wpItem.active').removeClass("active");
                $('#' + wp.id).addClass("active");
            });
        }
        map.addLayer(wp.marker);
        wayPointsList.push(wp);
    });
    currID = wps.length + 1;
    $("#wayPointsList").empty();
    wayPointsList.forEach(element => {
        var position = element.marker.getLatLng();
        $("#wayPointsList").append('<li class="list-group-item wpItem" id="' + element.id + '">' + posShow(position) + '<button class="btn btn-danger deleteWP" type="button">-</button></li>');
    });

    $(".deleteWP").click(deleteWPf);
}

/// Initialize

wayPointsList = [
    {
        latlong: [48.370954, -4.480665],
        id: "id1",
        currid: 5
    },
    {
        latlong: [48.380, -4.480665],
        id: "id2",
        currid: 5
    },
    {
        latlong: [48.370954, -4.4850],
        id: "id3",
        currid: 5
    },
    {
        latlong: [48.370954, -4.475],
        id: "id4",
        currid: 5
    }
];

updateWPList(wayPointsList);

var polyline = L.polyline([], { color: 'red' }).addTo(map);

var decorator = L.polylineDecorator(polyline, {
    patterns: [
        // defines a pattern of 10px-wide dashes, repeated every 20px on the line
        {offset: 0, repeat: 20, symbol: L.Symbol.arrowHead({pixelSize: 15, polygon: false, pathOptions: {stroke: true}})}
    ]
}).addTo(map);

updatePath();

/*
m.slideTo([48.864433, 2.371324], {
    duration: 3000
});

// or just set rotation with method
m.setRotationAngle(65);*/
/*
var marker = new L.marker(curLocation, {
    draggable: 'true'
});

marker.on('dragend', function (event) {
    var position = marker.getLatLng();
    marker.setLatLng(position, {
        draggable: 'true'
    }).bindPopup(position).update();
    console.log(position);
});*/

//map.addLayer(marker);