$("#navStats").addClass("active")

var compass = new RadialGauge({
    renderTo: 'compass',
    width : 300,
    height: 300,
    minValue: 0,
    maxValue: 360,
    majorTicks: [
        "N",
        "NE",
        "E",
        "SE",
        "S",
        "SW",
        "W",
        "NW",
        "N"
    ],
    minorTicks: 22,
    ticksAngle: 360,
    startAngle: 180,
    strokeTicks: false,
    highlights: false,
    colorPlate: "#222",
    colorMajorTicks: "#f5f5f5",
    colorMinorTicks: "#ddd",
    colorNumbers: "#ccc",
    colorNeedle: "rgba(240, 128, 128, 1)",
    colorNeedleEnd: "rgba(255, 160, 122, .9)",
    valueBox: false,
    valueTextShadow: false,
    colorCircleInner: "#fff",
    colorNeedleCircleOuter: "#ccc",
    needleCircleSize: 15,
    needleCircleOuter: false,
    animationRule: "linear",
    needleType: "line",
    needleStart: 75,
    needleEnd: 99,
    needleWidth: 3,
    borders: true,
    borderInnerWidth: 0,
    borderMiddleWidth: 0,
    borderOuterWidth: 10,
    colorBorderOuter: "#ccc",
    colorBorderOuterEnd: "#ccc",
    colorNeedleShadowDown: "#222",
    borderShadowWidth: 0,
    animationDuration: 1000
}).draw();

var windspd = new RadialGauge({
    renderTo: 'windspd',
    width : 300,
    height: 300,
    title:"WSPD",
    units: "kt",
    minValue: 0,
    maxValue: 22,
    majorTicks: [
        "0",
        "2",
        "4",
        "6",
        "8",
        "10",
        "12",
        "14",
        "16",
        "18",
        "20",
        "22"
    ],
    minorTicks: 2,
    strokeTicks: true,
    highlights: [
        {
            "from": 16,
            "to": 22,
            "color": "rgba(200, 50, 50, .75)"
        }
    ],
    colorPlate: "#eef",
    borderShadowWidth: 0,
    borders: false,
    needleType: "arrow",
    needleWidth: 2,
    valueBox: false,
    needleCircleSize: 7,
    needleCircleOuter: true,
    needleCircleInner: false,
    animationDuration: 1500,
    animationRule: "linear"
}).draw();

var speed = new RadialGauge({
    renderTo: 'speed',
    width : 300,
    height: 300,
    title:"GSPD",
    units: "kt",
    minValue: 0,
    maxValue: 22,
    majorTicks: [
        "0",
        "2",
        "4",
        "6",
        "8",
        "10",
        "12",
        "14",
        "16",
        "18",
        "20",
        "22"
    ],
    minorTicks: 2,
    strokeTicks: true,
    highlights: [
        {
            "from": 16,
            "to": 22,
            "color": "rgba(200, 50, 50, .75)"
        }
    ],
    colorPlate: "#fff",
    borderShadowWidth: 0,
    borders: false,
    needleType: "arrow",
    needleWidth: 2,
    valueBox: false,
    needleCircleSize: 7,
    needleCircleOuter: true,
    needleCircleInner: false,
    animationDuration: 1500,
    animationRule: "linear"
}).draw();

var heeling = new RadialGauge({
    renderTo: 'heeling',
    width: 300,
    height: 300,
    units: "HEELING",
    minValue: -60,
    startAngle: 180,
    ticksAngle: 120,
    valueBox: false,
    maxValue: 60,
    majorTicks: [
        "-60",
        "-40",
        "-20",
        "0",
        "20",
        "40",
        "60"
    ],
    minorTicks: 4,
    strokeTicks: true,
    highlights: [
        {
            "from": -60,
            "to": -40,
            "color": "rgba(200, 50, 50, .75)"
        },
        {
            "from": 40,
            "to": 60,
            "color": "rgba(200, 50, 50, .75)"
        }
    ],
    colorPlate: "#fff",
    borderOuterWidth: 10,
    colorBorderOuter: "#ccc",
    borderShadowWidth: 0,
    borders: true,
    needleType: "arrow",
    needleWidth: 2,
    needleCircleSize: 7,
    needleCircleOuter: true,
    needleCircleInner: false,
    animationDuration: 750,
    animationRule: "linear",
    animationTarget: "plate"
}).draw();