/**
 *  Functions that interact with the server via AJAX.
 */

/**
 *  Pack the coordinates into something that thrift can understand.
 *  @param [in] data Object containing leaflet coordinates 
 *  @return Array of coordinates as [lat,lon] array pairs
 */
function packageCoordinates(data) {
  var newdata = [];
  if (typeof data != 'undefined') {
		$.each(data, function(index, val) {
			lat = val.getLatLng().lat;
			lng = val.getLatLng().lng;
			newdata.push([lat,lng]);
		});
	}
  return newdata;
}

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
		data: {
			'action': action,
			'data'   : data
		},
		success: function(response) {
			$('#response').html(response);
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
 *  Begin the autonomous lawnmower search pattern
 */
function beginAuto() {
	if (!canEdit && bounds.length == 2) {
    ajaxSend('updateWaypoints', packageCoordinates(bounds)).
      success(function () {
        ajaxSend('beginAuto');
    });
	}
}

/**
 *  Begin the autnonomous waypoints navigation
 */
function beginManual() {
	if (!canEdit && markers.length > 0) {
		ajaxSend('updateWaypoints', packageCoordinates(markers)).
      success(function () {
        ajaxSend('beginManual');
    });
	}
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
 *  Begin the object tracking function (using camera)
 */
function beginObjectTracking(method) {
  ajaxSend('beginObjectTracking', method);
}

/**
 *  Set the camera mode
 */
function setCameraMode() {
	//yuck
	var modes = [0,1,2,3,999];
  var modenames = {0 : "No processing", 1 : "Centre of mass", 2 : "Camshift",
                   3 : "Connected components", 999 : "Colour training"}
	if (typeof setCameraMode.currentModeIndex === "undefined") {
		setCameraMode.currentModeIndex = 4;
	}

	ajaxSend('setCameraMode', modes[setCameraMode.currentModeIndex]).
    success(function (ret) {
      var mode = parseInt(ret, 10);
      if (mode in modenames) {
        $("#camera-mode").text(modenames[mode]);
      } else {
        $("#camera-mode").text("Unknown mode");
      }
  });
	setCameraMode.currentModeIndex = (setCameraMode.currentModeIndex + 1) % modes.length;
}

function setCameraLearningSize(decrease) {
	ajaxSend('setCameraLearningSize', decrease ? 1 : 0);
}

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

function toggleLearningThreshold() {
  if (typeof toggleLearningThreshold.show === "undefined") {
		toggleLearningThreshold.show = false;
	}
  toggleLearningThreshold.show = !toggleLearningThreshold.show;
  ajaxSend('showLearningThreshold', toggleLearningThreshold.show ? 1 : 0);
}

function requestSettings() {
  return ajaxSend('requestSettings').success(function (data) {
    var ret = $.parseJSON(data);
    var form = $("#settings-space");
    form.empty();
    
    for (var key in ret) {
      var family = $("<h2/>", {text : key});
      var familyData = ret[key];
      form.append(family);
      var table = $("<table/>");
      var count = 0;
      var row = null;
      
      for (var opt in familyData) {
        if (!(count++ % 5)) {
          if (row) {
            table.append(row);
          }
          row = $("<tr/>");
        }
        
        var optid = key + "." + opt;
        var group = $("<td/>").append($("<div/>", {class : "form-group"}).append(
                      $("<label/>", {"for" : optid, "text" : opt})).append(
                      $("<input/>", {"id" : optid, "type" : "text", "class" : "form-control", "value" : familyData[opt]})));
        row.append(group);
      }
      
      if (row) {
        table.append(row);
      }
      form.append(table);
    }
  });
}

function updateSettings() {
  $("#settings-editor :input").prop("disabled", true);
  
  var sub = {};
  $("#settings-editor :input[type=text]").each(function () {
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
  
  return ajaxSend('updateSettings', JSON.stringify(sub)).complete(function () {
    requestSettings().complete(function () {
      $("#settings-editor :input").prop("disabled", false);
    });
  })
}
