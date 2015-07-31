/**
 * Autorun functions for the main webpage.
 */
 
/**
 * Start the geolocation updater for the 'follow me' function.
 */
function startGeoLocator() {
  if (navigator.geolocation) {
    navigator.geolocation.watchPosition(updateUserPosition);
  }
}

/**
 * Initialise the camera and the hue calibration sliders.
 */
function cameraInit() {
  var url = "http://" + document.domain + ":5000/?action=stream";
  $("#camera-secondary").html("<img id='camera-secondary-img' src='" + url + "'/>");
  sliderify("#cal-hue", -20, 20, -360, 360);
  sliderify("#cal-sat", 97, 255, 0, 255);
  sliderify("#cal-val", 127, 255, 0, 255);
}

/**
 * Worker thread to continuously poll the server for its status.
 */
function statusWorker(copter_yaw) {
  if ( typeof userMarker !== 'undefined' ) {
    lat = userMarker.getLatLng().lat;
    lon = userMarker.getLatLng().lng;
    
    data = {
      'action': 'requestAll',
      'lat': lat,
      'lon': lon
    }
  } else {
    data = {
      'action': 'requestAll'
    }
  }
  
  $.ajax({
    type: "POST",
    dataType: "json",
    url:'ajax-thrift.php',
    data: data,
    success: function(data) {
      $("#status").html(data.status);
      
      $("#bearing").html("Facing " + Math.round(data.bearing) + "&deg;");
      copter_yaw.setHeading(data.bearing); //WE HAVE BEARING, HEADING AND YAW!
      
      var latlng = L.latLng(data.lat, data.lon);
      copterMarker.setLatLng(latlng).update();
      
      if (pathEnabled) updatePath(latlng);
    },
    error: function() {
      $("#status").html("ERROR: No connection to flight control program.");
    },
    complete: function() {
      setTimeout(statusWorker, 1200, copter_yaw);
    }
  });
}

/**
 * Autorunned functions. 
 */
$(document).ready(function () {
  startGeoLocator();
  cameraInit();
  
  var copter_yaw = $.flightIndicator('#copter_yaw', 'heading', {heading:150, showBox:true});
  statusWorker(copter_yaw);
  
  //Switch back to map view if still on the settings page and the user navigates away.
  $("#menu-button-holder button").click(function () {
    if ($("#opts-main").is(":visible")) {
      toggleMain("#opts-main");
      $("#settings-opts").removeClass('orange-toggle');
    }
  });
  
});