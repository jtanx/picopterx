/**
 * @file ajax.js
 * @brief Functions to interact with the server (via AJAX calls)
 */

/**
 *  Sends the command to the server
 *  @param [in] action The string of the command to be sent
 *  @param [in] data The data to be sent
 *  @return The AJAX promise
 */
function ajaxSend(action, data) {
  return $.ajax({
    type: "POST",
    url: "ajax-thrift.php",
    timeout: 2000,
    data: {
      'action': action,
      'data'   : data
    },
    success: function(response) {
      //$('#response').html(response);
    }
  });
}

/**
 * Populates the detected object list.
 */
function getDetectedObjects() {
  return $.ajax({
    type: "GET",
    url: "ajax-review.php",
    timeout: 2000,
    success: function(response) {
      var ret = $.parseJSON(response);
      $('#detection-review').empty();
      $.each(ret, function (index, detection) {
        detection.timestamp = new Date(detection.timestamp);
        $.each(detection.detected, function(idx, v) {
          v.timestamp = new Date(v.timestamp);
        });
        
        var opt = $('<option>', {
            value: detection.log,
            text: detection.log
        }).data("detection", detection);
        $('#detection-review').append(opt);
        console.log(detection);
      });
    }
  });
}

/**
 *  Send the all stop command
 */
function allStop() {
  $("#allstop").trigger("allstop");
  return ajaxSend('allStop');
}

/**
 * Perform a take-off.
 * @param [in] alt The altitude above ground to take-off to.
 */
function takeOff(alt) {
  return ajaxSend('beginTakeoff', alt);
}

/**
 * Perform a return to launch (RTL).
 */
function returnToLaunch() {
  return ajaxSend('beginReturnToLaunch');
}

/**
 * Begin navigation by waypoints.
 */
function beginWaypoints() {
  $("#map-canvas").copterMap('getActiveMarkerCoordinates', function (pattern, coords) {
    $("#map-canvas").copterMap('getExclusionZones', function(zones) {
      var mode;
      
      if (coords.length > 0 && pattern === "manual") {
        mode = 0;
      } else if (coords.length == 2&& pattern == "lawnmower") {
        mode = 1;
      } else if (coords.length >= 2 && pattern == "spiral") {
        mode = $("#wpt-spiraldir").is(":checked") ? 3 : 2;
      }
      
      if (typeof mode !== "undefined") {
        ajaxSend('updateWaypoints', coords).success(function () {
          ajaxSend('updateExclusions', zones).success(function () {
            ajaxSend('beginWaypoints', mode);
          });
        });
      }
    });
  });
}

/**
 *  Begin the autnonomous user tracking function
 */
function beginUserTracking() {
  if ((navigator.geolocation) && $("#user-tracker").hasClass("btn-primary")) {
    ajaxSend('beginUserTracking');
  } else {
    alert("You are not in the geofence. Not activating user tracking!");
  }
}

/**
 * Begin the taking pictures thread.
 */
function beginTakingPictures() {
    ajaxSend('beginTakingPictures');
}

/**
 *  Begin the Environmental Mapping
 */
function beginUserMapping(radius) {
  if (typeof radius !== "undefined" && radius > 2 && radius < 20) {
    ajaxSend('beginUserMapping', radius);
  } else {
    ajaxSend('beginUserMapping', -1);
  }
}

/**
 *  Begin the object tracking function (using camera)
 */
function beginObjectTracking(method) {
  ajaxSend('beginObjectTracking', method);
}

/**
 * Set the camera mode
 * @param [in] mode The mode to set to.
 */
function setCameraMode(mode) {
  return ajaxSend('setCameraMode', mode).
    success(function (ret) {
      var mode = parseInt(ret, 10);
      $("#camera-mode").val(mode);
  });
}

/**
 * Set the learning size for auto learning the colour.
 * @param [in] decrease true to decrease, false to increase size.
 */
function setCameraLearningSize(decrease) {
  ajaxSend('setCameraConfig', JSON.stringify({"CAMERA_STREAM" : {"SET_LEARNING_SIZE" : decrease}}));
}

/**
 * Parse to ints.
 */
function toInts(val) {
  return parseInt(val, 10);
};

/**
 * Manually set the Hue, Saturation and Value parameters.
 */
function setCameraLearningValues() {
  var csp = $("#camera-thresh-colourspace option:selected").val();
  var ret={};
  
  if (csp === "csp-hsv") {
    var h = $("#cal-hue").val().map(toInts);
    var s = $("#cal-sat").val().map(toInts);
    var v = $("#cal-val").val().map(toInts);

    ret["THRESH_COLOURSPACE"] = 0;
    ret["MIN_HUE"] = h[0]; ret["MAX_HUE"] = h[1];
    ret["MIN_SAT"] = s[0]; ret["MAX_SAT"] = s[1];
    ret["MIN_VAL"] = v[0]; ret["MAX_VAL"] = v[1];
    
  } else if (csp === "csp-ycbcr") {
    var y = $("#cal-y").val().map(toInts);
    var cb = $("#cal-cb").val().map(toInts);
    var cr = $("#cal-cr").val().map(toInts);

    ret["THRESH_COLOURSPACE"] = 1;
    ret["MIN_Y"] = y[0]; ret["MAX_Y"] = y[1];
    ret["MIN_Cb"] = cb[0]; ret["MAX_Cb"] = cb[1];
    ret["MIN_Cr"] = cr[0]; ret["MAX_Cr"] = cr[1];   
  }

  ajaxSend('setCameraConfig', JSON.stringify({"CAMERA_STREAM" : ret})).success(function() {
    ajaxSend('requestCameraConfig').success(function(ret) {
      updateDisplayedCameraConfig($.parseJSON(ret)["CAMERA_STREAM"]);
    });
  });
}

/**
 * Perform camera auto learning.
 */
function doCameraAutoLearning() {
  var csp = $("#camera-thresh-colourspace option:selected").val();
  var ret;
  if (csp === "csp-ycbcr") {
    var y = $("#cal-y").val().map(toInts);
    ret = ajaxSend('setCameraConfig', JSON.stringify({"CAMERA_STREAM" : 
      {
        "THRESH_COLOURSPACE" : 1,
        "MIN_Y" : y[0],
        "MAX_Y" : y[1]
      }
    }));
  } else {
    var s = $("#cal-sat").val().map(toInts);
    var v = $("#cal-val").val().map(toInts);
    
    ret = ajaxSend('setCameraConfig', JSON.stringify({"CAMERA_STREAM" : 
      {
        "THRESH_COLOURSPACE" : 0,
        "MIN_SAT" : s[0],
        "MAX_SAT" : s[1],
        "MIN_VAL" : v[0],
        "MAX_VAL" : v[1]
      }
    }));
  }
  
  ret.success(function () {
    ajaxSend('doCameraAutoLearning').success(function() {
      ajaxSend('requestCameraConfig').success(function(ret) {
        updateDisplayedCameraConfig($.parseJSON(ret)["CAMERA_STREAM"]);
      });
    });
  });
}

/**
 * Toggle the display of the thresholded image when in calibration mode.
 */
function toggleLearningThreshold() {
  if (typeof this.show === "undefined") {
    this.show = false;
  }
  this.show = !this.show;
  ajaxSend('setCameraConfig', JSON.stringify({"CAMERA_STREAM" : {"SHOW_BACKEND" : this.show}}));
}


/**
 * Request settings from the server and populate the form with it.
 * @return The AJAX promise on the server request.
 */
$.fn.requestSettings = function() {
  var form = $(this);
  return ajaxSend('requestSettings').success(function (data) {
    var ret = $.parseJSON(data);
    form.empty();

    for (var key in ret) {
      var family = $("<h2/>", {text : key});
      var familyData = ret[key];
      form.append(family);
      var table = $("<div/>", {"class" : "family-group"});
      var count = 0;
      var row = null;

      for (var opt in familyData) {
        if (!(count++ % 4)) {
          if (row) {
            table.append(row);
          }
          row = $("<div/>", {"class" : "family-row row"});
        }

        var optid = key + "." + opt;
        var group = $("<div/>", {class : "form-group col-md-3"}).append(
                      $("<label/>", {"for" : optid, "text" : opt})).append(
                      $("<input/>", {"id" : optid, "type" : "text", "class" : "form-control", "value" : familyData[opt]}));
        row.append(group);
      }

      if (row) {
        table.append(row);
      }
      form.append(table);
    }
  });
}

/**
 * Take all settings and send them to the server. Use the response from
 * the server to then update what settings are currently shown.
 * @return The AJAX promise on the request to the server.
 */
$.fn.updateSettings = function () {
  var form = $(this);
  var sub = {};

  form.find(":input").prop("disabled", true);
  form.find(":input[type=text]").each(function () {
    //This will break if we have a family name with a . in it.
    var brk = $(this).attr('id').split(".");
    var val = $(this).val();
    var family = brk[0], opt = brk[1];

    if (!(family in sub)) {
      sub[family] = {};
    }

    if (val === 'true') {
      sub[family][opt] = true;
    } else if (val === 'false') {
      sub[family][opt] = false;
    } else if (val === '') {
      //pass
    } else if (!isNaN(val)) {
      sub[family][opt] = parseFloat(val);
    } else {
      sub[family][opt] = val;
    }
  });

  return ajaxSend('updateSettings', JSON.stringify(sub)).complete(function (data,a,b,c,d) {
    form.requestSettings().complete(function (data) {
      form.find(":input").prop("disabled", false);
    });
  });
}
