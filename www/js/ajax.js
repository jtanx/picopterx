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
    timeout: 1200,
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
 *  Send the all stop command
 */
function allStop() {
  ajaxSend('allStop');
}

/**
 * Begin navigation by waypoints.
 */
function beginWaypoints() {
  $("#map-canvas").copterMap('getActiveMarkerCoordinates', function (pattern, coords) {
    if (coords.length > 0 && pattern === "manual") {
      ajaxSend('updateWaypoints', coords).success(function () {
        ajaxSend('beginManual');
      });
    } else if (coords.length == 2&& pattern == "lawnmower") {
      ajaxSend('updateWaypoints', coords).success(function () {
        ajaxSend('beginAuto');
      });
    }
  });
}

/**
 *  Begin the autnonomous user tracking function
 */
function beginUserTracking() {
  if ((navigator.geolocation)) {
    ajaxSend('beginUserTracking');
  }
}

/**
 *  Begin the Environmental Mapping
 */
function beginUserMapping() {
    ajaxSend('beginUserMapping');
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
  ajaxSend('setCameraLearningSize', decrease ? 1 : 0);
}

/**
 * Perform camera auto learning.
 */
function doCameraAutoLearning() {
  ajaxSend('doCameraAutoLearning').success(function(ret) {
    ret = $.parseJSON(ret);
    if (ret["MIN_HUE"] > ret["MAX_HUE"]) {
      ret["MIN_HUE"] = ret["MIN_HUE"] - 360;
    }
    $("#cal-hue").val([ret["MIN_HUE"], ret["MAX_HUE"]]);
    $("#cal-sat").val([ret["MIN_SAT"], ret["MAX_SAT"]]);
    $("#cal-val").val([ret["MIN_VAL"], ret["MAX_VAL"]]);
  });
}

/**
 * Manually set the Hue, Saturation and Value parameters.
 */
function setCameraLearningValues() {
  function toInts(val) {
    return parseInt(val, 10);
  };

  var ret={};
  var h = $("#cal-hue").val().map(toInts);
  var s = $("#cal-sat").val().map(toInts);
  var v = $("#cal-val").val().map(toInts);

  ret["MIN_HUE"] = h[0]; ret["MAX_HUE"] = h[1];
  ret["MIN_SAT"] = s[0]; ret["MAX_SAT"] = s[1];
  ret["MIN_VAL"] = v[0]; ret["MAX_VAL"] = v[1];

  if (ret["MIN_HUE"] < 0) {
    ret["MIN_HUE"] += 360;
  }

  ajaxSend('setCameraLearningValues', ret).success(function(ret) {
    ret = $.parseJSON(ret);
    if (ret["MIN_HUE"] > ret["MAX_HUE"]) {
      ret["MIN_HUE"] = ret["MIN_HUE"] - 360;
    }
    $("#cal-hue").val([ret["MIN_HUE"], ret["MAX_HUE"]]);
    $("#cal-sat").val([ret["MIN_SAT"], ret["MAX_SAT"]]);
    $("#cal-val").val([ret["MIN_VAL"], ret["MAX_VAL"]]);
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
  ajaxSend('showLearningThreshold', this.show ? 1 : 0);
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