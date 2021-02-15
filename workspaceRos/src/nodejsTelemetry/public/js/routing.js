$("#navRouting").addClass("active")
var haveSafeZone = false;
var staticWaypoints = [];
var polys = [];

/// Socket.io
socket.on("staticWP", function(data) {
    console.log(data);
    currID = data.length;
    updateStaticWPList(data);
    socket.emit('gotWP', null);
});

socket.emit("getStaticWP");

socket.on("currentTarget", function (data) {
    var id = Number(data.data);
    if(id && id != currWP) {
        console.log(id);
        staticWaypoints[currWP].marker.setIcon(new L.Icon.Default());
        staticWaypoints[id].marker.setIcon(targetIcon);
        currWP = id;
    }
});

socket.on("newPolys", function (data) {
    console.log(data);
    polys.forEach(element => {
        map.removeLayer(element);
    });
    polys.length = 0;
    data.forEach(element => {
        if(!element.isObstacle){
            haveSafeZone = true;
            $("#addPolygonEndSafe").attr("disabled", "true");
        }
        var p = L.polygon(element.latlng);
        p.setStyle({color: element.isObstacle ? "red" : "green"});
        polys.push({
            polygon: p,
            isObstacle: element.isObstacle,
            id: polys[polys.length-1]==undefined ? 0 : polys[polys.length-1].id + 1
        });
        p.bindTooltip("id : " + polys[polys.length-1].id, {permanent: false, direction:"center"}).openTooltip()
    });
    updatePolyList();

    socket.emit('gotPolys', null);
});

const EPSILON = 0.00000000001
const EARTH_RADIUS = 6371000.
var lat0 = 48.431775
var lon0 = -4.615529

var cargoIcon = new L.Icon.Default();
cargoIcon.options.shadowSize = [0, 0];
cargoIcon.options.iconUrl = "cargo.png";
var size = .6;
cargoIcon.options.iconAnchor = [30 * size, 60 * size];
cargoIcon.options.iconSize = [61 * size, 100 * size];

var cargoMarkers = [];
var updateCargos = function() {
    cargoMarkers.forEach(element => {
        map.removeLayer(element);
    });
    cargoMarkers.length = 0;
    cargos.forEach(element => {
        var clat = element.y*180./Math.PI/EARTH_RADIUS+state.lat0
        var clon;
        if (Math.abs(lat-90.) < EPSILON || Math.abs(lat+90.) < EPSILON)
        {
            clon = 0
        } else {
            clon = (element.x/EARTH_RADIUS)*(180./Math.PI)/Math.cos((Math.PI/180.)*(clat))+state.lon0
        }
        var movingCargo = new L.marker([clat, clon], {
            icon: cargoIcon
        }).addTo(map);
        var angle = element.theta;
        movingCargo.setRotationAngle(angle);
        cargoMarkers.push(movingCargo);
    });
}

setInterval(function() {
    var angle = -state.heading*90 / Math.PI;
    myMovingMarker.setRotationAngle(angle);


    lat = state.y*180./Math.PI/EARTH_RADIUS+state.lat0
    if (Math.abs(lat-90.) < EPSILON || Math.abs(lat+90.) < EPSILON)
    {
        lon = 0
    } else {
        lon = (state.x/EARTH_RADIUS)*(180./Math.PI)/Math.cos((Math.PI/180.)*(lat))+lon0
    }

    myMovingMarker.slideTo([lat, lon], {
        duration: .5
    });

    updateCargos();
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
}).setView([48.431775, -4.615529], 13);

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

var routingTarget = new L.marker([48.4305, -4.6127], {
    icon: targetIcon,
    draggable: 'true'
}).addTo(map);

routingTarget.on('dragend', function (event) {
    var position = routingTarget.getLatLng();
    routingTarget.setLatLng(position, {
        draggable: 'true'
    }).update();
    latlong = {
        lat:position.lat, 
        lon:position.lng
    };
    socket.emit('routingTarget', latlong)
});

var currPoly;
var currPolyLatLngs = [];
var updateCurrPoly = function() {
    if(currPoly) {
        map.removeLayer(currPoly);
    }
    currPoly = L.polygon(currPolyLatLngs, {color: 'blue'}).addTo(map);
}
var addingPoints = false;
map.on('click', function (e) {
    $('.wpItem.active').removeClass("active");
    if (addingPoints) {
        var latlong = e.latlng;
        currPolyLatLngs.push(latlong);
        updateCurrPoly();
    }
});

var currID = 5;

///JQuery
$('.col-md-3').click(function() {
    $('.wpItem.active').removeClass("active");
});

$('#addPointsModal').collapse({
    toggle: false
});

$('#addPolygonEndSafe').click(function (event) {
    addingPoints = false;
    currPoly.setStyle({color: 'green'});
    polys.push({
        polygon: currPoly,
        isObstacle: false,
        id: polys[polys.length-1]==undefined ? 0 : polys[polys.length-1].id + 1
    });
    currPoly.bindTooltip("id : " + polys[polys.length-1].id, {permanent: false, direction:"center"}).openTooltip()
    currPoly = undefined;
    $('#addPointsModal').collapse('hide');
    haveSafeZone = true;
    $("#addPolygonEndSafe").attr("disabled", "true");
    updatePolyList();
});

$('#addPolygonEndObstacle').click(function (event) {
    addingPoints = false;
    currPoly.setStyle({color: 'red'});
    polys.push({
        polygon: currPoly,
        isObstacle: true,
        id: polys[polys.length-1]==undefined ? 0 : polys[polys.length-1].id + 1
    });
    currPoly.bindTooltip("id : " + polys[polys.length-1].id, {permanent: false, direction:"center"}).openTooltip()
    currPoly = undefined;
    $('#addPointsModal').collapse('hide');
    updatePolyList();
});


$(".addPoly").click(function (event) {
    $('#addPointsModal').collapse('show');
    addingPoints = true;
    currPolyLatLngs.length = 0;
    updateCurrPoly();
    currPolyLatLngs.length = 0;
});

$(".submitPolys").click(function (event) {
    polygons = [];
    polys.forEach(element => {
        ll = [];
        element.polygon._latlngs[0].forEach(latlongs => {
            if (latlongs.lat != undefined && latlongs.lng != undefined) {
                ll.push([latlongs.lat, latlongs.lng]);
            }
        });
        
        polygons.push({
            latlng: ll,
            isObstacle: element.isObstacle
        })
    });
    console.log(polygons);
    socket.emit('userPolys', polygons)
});


var updateStaticWPList = function (wps) {
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


var updatePolyList = function() {
    $("#polyList").empty();
    polys.forEach(element => {
        $("#polyList").append('<li class="list-group-item wpItem" id="' + element.id + '"> Polygon with ID : ' + element.id + ' <img src="images/'+ (element.isObstacle ? "redPoly" : "greenPoly") +'.png"> <button class="btn btn-danger deletePoly" id="deletePoly'+element.id+'" type="button">-</button></li>');
        $("#deletePoly"+element.id).click(function (event) {
            $(this).closest("li").remove();
            console.log(element.isObstacle);
            if(!element.isObstacle) {
                console.log("safe zone delete");
                haveSafeZone = false;
                $("#addPolygonEndSafe").removeAttr("disabled");
            }
            map.removeLayer(element.polygon);
            polys.splice(polys.indexOf(element), 1);
        });
    });
    polys.forEach(element => {
        map.addLayer(element.polygon);
    });
}

/// Initialize

var polylineStatic = L.polyline([]).setStyle({
    color: 'grey'
});
polylineStatic.addTo(map);

var decoratorStatic = L.polylineDecorator(polylineStatic, {
    patterns: [
        // defines a pattern of 10px-wide dashes, repeated every 20px on the line
        {offset: 0, repeat: 20, symbol: L.Symbol.arrowHead({pixelSize: 7, polygon: false, pathOptions: {color: "#555555", stroke: true}})}
    ]
}).addTo(map);

function drawPolys(newPolys) {
    //newPolygons is an array of {latlng:..., isObstacle:..., id:...}
    polys.forEach(element => {
        map.removeLayer(element);
    });
    newPolys.forEach(element => {
        var pl = new L.Polygon(element.latlng);
        var polyColor = element.isObstacle ? "red" : "green";
        pl.setStyle({color: polyColor});
        var new_id = polys.length;
        polys.push({
            polygon: pl,
            id: new_id,
            isObstacle: element.isObstacle
        });
        pl.bindTooltip("id : " + new_id, {permanent: false, direction:"center"}).openTooltip()
        
        updatePolyList();
    });
}

function download(content, fileName, contentType) {
    var a = document.createElement("a");

    var file = new Blob([JSON.stringify(content)], {type: contentType});
    a.href = URL.createObjectURL(file);
    a.download = fileName;
    a.click();
}
$('.downloadBtn').click(function(event) {
    var pl = [];
    polys.forEach(element => {
        var ll = [];
        element.polygon._latlngs[0].forEach(latlongs => {
            if (latlongs.lat != undefined && latlongs.lng != undefined) {
                ll.push([latlongs.lat, latlongs.lng]);
            }
        });
        pl.push({
            id: element.id,
            isObstacle: element.isObstacle,
            latlng: ll
        })
    });

    download(pl, 'polygons.json', 'application/json');
});

function readSingleFile(e) {
    var file = e.target.files[0];
    if (!file) {
      return;
    }
    var reader = new FileReader();

    reader.onload = function(e) {
        var contents = e.target.result;
        var pl = JSON.parse(contents);
        drawPolys(pl);
        updatePolyList();
    };
    reader.readAsText(file);
  }

  
$("#file-input").on('change', readSingleFile);

var updateLogs = function() {
    //$(".terminal-home")
}

logs = [];
// socket.on("rosInfo", function (data) {
//     console.log(data);
//     logs.push(data);
//     logs.length = 10;
//     updateLogs();
//})