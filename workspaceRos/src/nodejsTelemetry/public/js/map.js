$("#navMission").addClass("active")


var wayPointsList = [];
var staticWaypoints = [];

/// Socket.io
socket.on("yourWP", function(data) {
    currID = data.length;
    updateStaticWPList(data);
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

socket.on("currentTarget", function (data) {
    var id = Number(data.data);
    if(id && id != currWP) {
        console.log(id);
        staticWaypoints.forEach(element => {
            element.marker.setIcon(new L.Icon.Default());
        });
        staticWaypoints[id].marker.setIcon(targetIcon);
        $("#wpid").text(data.data);
        currWP = id;
    }
});

const EPSILON = 0.00000000001
const EARTH_RADIUS = 6371000.
const lat0 = 48.431775
const lon0 = -4.615529


setInterval(function() {
    var angle = -state.heading*90 / Math.PI;
    myMovingMarker.setRotationAngle(angle);


    lat = state.y*180./Math.PI/EARTH_RADIUS+lat0
    if (Math.abs(lat-90.) < EPSILON || Math.abs(lat+90.) < EPSILON)
    {
        lon = 0
    } else {
        lon = (state.x/EARTH_RADIUS)*(180./Math.PI)/Math.cos((Math.PI/180.)*(lat))+lon0
    }

    myMovingMarker.slideTo([lat, lon], {
        duration: .5
    });
}, 500);


///Leaflet.js
/*var baseMap = L.tileLayer('http://a.tile.openstreetmap.fr/hot/{z}/{x}/{y}.png', {
    attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
});*/
var baseMap = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
	attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community'
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

var targetIcon = L.icon({
    iconUrl: 'css/images/target.png',
    shadowUrl: 'css/images/target.png',

    iconSize:     [40, 40], // size of the icon
    shadowSize:   [0, 0], // size of the shadow
    iconAnchor:   [20, 20], // point of the icon which will correspond to marker's location
    shadowAnchor: [20, 20],  // the same for the shadow
    popupAnchor:  [20, 0] // point from which the popup should open relative to the iconAnchor
});

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

var updateStaticWPList = function (wps) {
    console.log(wps);
    //wps must contain an id and a latlong array at the bare minimum
    staticWaypoints.forEach(element => {
        if (element.marker) {
            map.removeLayer(element.marker);
        }
    });

    staticWaypoints = [];

    wps.forEach(wp => {
        if (!wp.marker) {
            var marker = new L.marker(wp.latlong, {
                draggable: 'false'
            });
            
            wp.marker = marker;

        }
        map.addLayer(wp.marker);
        staticWaypoints.push(wp);
        wp.marker.dragging.disable();
        wp.marker.setZIndexOffset(-1);
    });
    var latlongs = [];
    staticWaypoints.forEach(element => {
        latlongs.push(element.latlong);
    });
    polylineStatic.setLatLngs(latlongs);
    decoratorStatic.setPaths(polylineStatic);
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

var polyline = L.polyline([]).setStyle({
    color: 'blue'
});
polyline.addTo(map);

var polylineStatic = L.polyline([]).setStyle({
    color: 'grey'
});
polylineStatic.addTo(map);


var decorator = L.polylineDecorator(polyline, {
    patterns: [
        // defines a pattern of 10px-wide dashes, repeated every 20px on the line
        {offset: 0, repeat: 20, symbol: L.Symbol.arrowHead({pixelSize: 7, polygon: false, pathOptions: {stroke: true}})}
    ]
}).addTo(map);

var decoratorStatic = L.polylineDecorator(polyline, {
    patterns: [
        // defines a pattern of 10px-wide dashes, repeated every 20px on the line
        {offset: 0, repeat: 20, symbol: L.Symbol.arrowHead({pixelSize: 7, polygon: false, pathOptions: {color: "#555555", stroke: true}})}
    ]
}).addTo(map);

updatePath();

function download(content, fileName, contentType) {
    var a = document.createElement("a");

    wplist = []
    content.forEach(element => {
        wplist.push({
            latlong:element.latlong,
            id: element.id
        })
    });

    var file = new Blob([JSON.stringify(wplist)], {type: contentType});
    a.href = URL.createObjectURL(file);
    a.download = fileName;
    a.click();
}
$('.downloadBtn').click(function(event) {
    console.log('catch');
    download(wayPointsList, 'mission.json', 'application/json');
});
var f;
function readSingleFile(e) {
    var file = e.target.files[0];
    if (!file) {
      return;
    }
    var reader = new FileReader();

    reader.onload = function(e) {
        var contents = e.target.result;
        updateWPList(JSON.parse(contents));
        updatePath();
    };
    reader.readAsText(file);
  }

  
$("#file-input").on('change', readSingleFile);

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