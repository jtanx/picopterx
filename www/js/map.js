var uwa = [-31.979839, 115.817546];
var markers = [];
var bounds = [];
var copterMarker;
var userMarker;
var map = L.map('map-canvas').setView(uwa, 18);

var boundRect;
var pathLine;
var wptPathLine;

var popup = L.popup();

/* ************************************* INITIALISATION */

function initialise() {
	L.tileLayer('/tiles/sat/gs_{x}_{y}_{z}.jpg', {
		maxZoom: 21,
		minZoom: 17
	}).addTo(map);
	
	addCopter();
	addPath();
	addUser();
	
	wptPathLine = L.polyline( {color: 'blue'}).addTo(map);
}

$(function() {
    initialise();
});

/* ************************************* NEW MARKERS */

function addUser() {
	if (navigator.geolocation) {
		var userIcon = L.icon({
			iconUrl:	'/css/markers/marker-user.png',
			iconSize:	[32,37],
			iconAnchor:	[16,35]
		});
		
		userMarker =
			new L.marker( uwa, {
				draggable: false,
				icon: userIcon
			}).addTo(map);
	}
}

function addCopter() {
	var copterIcon = L.icon({
		iconUrl:	'/css/markers/helicopter.png',
		iconSize:	[32,37],
		iconAnchor:	[16,35]
	});
	
	copterMarker =
		new L.marker( uwa, {
			draggable: false,
			icon: copterIcon
		}).addTo(map);
}

function addMarker(data,loc,red) {
	if (red) {
		data.push(
			new L.marker( loc, {
				draggable: true,
				icon:	new L.NumberedDivIconRed({number: (data.length+1)})
			} ).addTo(map)
		);
	} else {
		data.push(
			new L.marker( loc, {
				draggable: false,
				icon:	new L.NumberedDivIcon({number: (data.length+1)})
			} ).addTo(map)
		);
	}

	var thisNo = data.length-1;	
	var thisMarker = data[thisNo];

	thisMarker.on('click', function(event) {
		if (canEdit) {
			map.removeLayer(thisMarker);
			data.splice( $.inArray(thisMarker,data) ,1 );
			hideRectangle();
		}
		updateWptPath();
	});
	
	thisMarker.on('drag', function(e) {
		if (canEditBounds) updateRectangle();
		if (canEditMarkers) updateWptPath();
	});
}

/* ************************************* MODIFICATIONS */

function updateUserPosition(position) {
	var latlng = L.latLng(position.coords.latitude, position.coords.longitude);
	userMarker.setLatLng(latlng).update();
}

function toggleMarkerRed(data) {
	tmpmarkers = data.slice();
	clearMarkers(data,false);
	$.each( tmpmarkers, function( index, value ){
		addMarker(data,value.getLatLng(), canEdit);
	});
}

function clearMarkers(data, ajax) {
	for (var i = 0; i < data.length; i++) {
		map.removeLayer(data[i]);
	}
	
	while(data.length > 0) {
		data.pop();
	}
	if (ajax) ajaxSend('resetWaypoints');
}

function updateRectangle() {
	if (bounds.length == 2) {
		map.removeLayer(boundRect);
		boundRect = L.rectangle(L.latLngBounds(bounds[0].getLatLng(), bounds[1].getLatLng()), {color: "#0066FF", weight: 1}).addTo(map);
	}
}

function updatePath(data) {
	pathLine.addLatLng(data);
}

/* ************************************* VISIBILITY */

function addPath() {
	pathLine = L.polyline(copterMarker.getLatLng(), {color: 'red'}).addTo(map);
}

function removePath() {
	map.removeLayer(pathLine);
}

function updateWptPath() {
	if (wptPathLine.getLatLngs().length > 0) map.removeLayer(wptPathLine);
	
	var data = [];
	
	$.each(markers, function(index, value) {
		data.push(markers[index].getLatLng());
	});
	
	wptPathLine = L.polyline(data, {color: 'blue'}).addTo(map);
}

function hideMarkers(data) {
	for (var i = 0; i < data.length; i++) {
		map.removeLayer(data[i]);
	}
}

function showMarkers(data) {
	for (var i = 0; i < data.length; i++) {
		data[i].addTo(map);
	}
}

function hideRectangle() {
	map.removeLayer(boundRect);
}

function showRectangle() {
	if (bounds.length == 2) boundRect = L.rectangle(L.latLngBounds(bounds[0].getLatLng(), bounds[1].getLatLng()), {color: "#0066FF", weight: 1}).addTo(map);
}

/* ************************************* CLICKETY-CLICK */

function onMapClick(e) {
	if (canEditMarkers) {
		addMarker(markers, e.latlng, true);
		updateWptPath();
	} else if (canEditBounds) {
		if (bounds.length == 0) {
			addMarker(bounds, e.latlng, true);
		
		} else if (bounds.length == 1) {
			addMarker(bounds, e.latlng, true);
			showRectangle();
		
		} else {
			updateRectangle();
		}
	}
}

map.on('click', onMapClick);

