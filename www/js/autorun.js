/**
 * Autorun functions for the main webpage.
 */

/**
 * Start the geolocation updater for the 'follow me' function.
 */
function startGeoLocator() {
  if (navigator.geolocation) {
    navigator.geolocation.watchPosition(function (pos) {
      $("#map-canvas").copterMap('updateUserPosition', pos.coords.latitude, pos.coords.longitude);
    });
  }
}

/**
 * Initialise the camera and the hue calibration sliders.
 */
function cameraInit() {
  var ua = window.navigator.userAgent;
  var msie = ua.indexOf("MSIE ");
  var simg = $("<img/>", {"id" : "camera-secondary-img"});
  var pimg = $("<img/>", {"id" : "camera-primary-img", "class" : "fill"});
  var url;

  /* IE does not support MJPG streaming... */
  if (msie > 0 || !!navigator.userAgent.match(/Trident.*rv\:11\./)) {
    url = "http://" + document.domain + ":5000/?action=snapshot";

    /* The number increment is to just force it to reload the image */
    (function updateImg(n) {
      simg.attr("src", url + "&n=" + n);
      pimg.attr("src", url + "&n=" + n);
      setTimeout(updateImg, 200, n+1);
    })(0);
  } else {
    url = "http://" + document.domain + ":5000/?action=stream";
  }

  simg.attr("src", url);
  pimg.attr("src", url);
  $("#camera-secondary").append(simg);
  $("#camera-primary").append(pimg);

  $("#cal-hue").sliderify(-20, 20, -180, 180);
  $("#cal-sat").sliderify(97, 255, 0, 255);
  $("#cal-val").sliderify(127, 255, 0, 255);
  $("#cal-y").sliderify(0, 255, 0, 255);
  $("#cal-cb").sliderify(86, 141, 0, 255);
  $("#cal-cr").sliderify(167, 255, 0, 255);

  ajaxSend('requestCameraMode').success(function (response) {
    var mode = parseInt(response, 10);
    if (mode >= 0) {
      $("#camera-mode").val(mode);
      if (mode == 999) {
        $("#camera-calibration-inputs").removeClass('hidden');
      }
    }
  });
  
  ajaxSend('requestCameraConfig').success(function(ret) {
    updateDisplayedCameraConfig($.parseJSON(ret)["CAMERA_STREAM"]);
  });
}

// FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME
// Hard-coded safety bounds.
var jamesOvalBounds = new L.latLngBounds(
    [[-31.9803622462528, 115.817576050758],[-31.9797547847258, 115.818262696266]]);
// FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME FIXME

/**
 * Worker thread to continuously poll the server for its status.
 * @param hud The structure containing the HUD objects.
 */
function statusWorker(hud) {
  var data = {'action': 'requestAll'};
  var userPosition = {};
  
  $("#map-canvas").copterMap('getUserPosition', userPosition);
  if (userPosition.hasOwnProperty('lat') && userPosition.hasOwnProperty('lon') &&
      jamesOvalBounds.contains(L.latLng(userPosition.lat, userPosition.lon))) {
        
    data.lat = userPosition.lat;
    data.lon = userPosition.lon;
    $("#user-tracker").removeClass("btn-default").addClass("btn-primary");
  } else {
    $("#user-tracker").removeClass("btn-primary").addClass("btn-default");
  }
  

  $.ajax({
    type: "POST",
    dataType: "json",
    url:'ajax-thrift.php',
    data: data,
    timeout: 2000,
    success: function(data) {
      $("#status-bar").text(data.status).removeClass("alert-danger alert-warning alert-success");
      if (data.status.indexOf("RTL") > -1) {
        $("#status-bar").addClass("alert-warning");
      } else {
        $("#status-bar").addClass("alert-success");
      }
      $("#map-canvas").copterMap('updateCopterPosition', data.lat, data.lon);
      hud.yaw.setHeading(data.yaw);
      hud.att.setRoll(data.roll);
      hud.att.setPitch(data.pitch);
      $("#copter-hud-secondary").html(
        "Pitch: "+ data.pitch.toFixed(1) + "&deg;" +
        " Roll: " + data.roll.toFixed(1) + "&deg;" +
        " Yaw: " + data.yaw.toFixed(1) + "&deg;" +
        " Alt: " + data.alt.toFixed(1) + "m" + 
        " Lidar: " + data.lidar.toFixed(1)  + "m");
    },
    error: function() {
      $("#status-bar")
        .text("ERROR: No connection to flight control program.")
        .removeClass("alert-success alert-warning")
        .addClass("alert-danger");
    },
    complete: function() {
      setTimeout(statusWorker, 1200, hud);
    }
  });
}

/**
 * Autorunned functions.
 */
$(document).ready(function () {
  $("#settings-editor").submit(function() {
    $("#settings-space").updateSettings();
    return false;
  });
  startGeoLocator();
  cameraInit();
  getDetectedObjects();

  var hud = {
    yaw : $.flightIndicator('#copter-yaw', 'heading', {showBox:false, size:150}),
    att : $.flightIndicator('#copter-att', 'attitude', {showBox:false, size: 150})
  };

  statusWorker(hud);
  initGamepads();
  gpPoller();
});