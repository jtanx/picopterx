/**
 * @file style.js
 * @brief JavaScript for controlling the appearance of the website.
 * @author Jeremy Tan 20933708
 */

/**
 *  Make a div into a slider
 *  @param [in] selector The jQuery selector
 *  @param [in] startmin Minimum starting value
 *  @param [in] startmax Maximum starting value
 *  @param [in] rangemin The range minimum
 *  @param [in] rangemax The range maximum
 *  @return The object
 */
$.fn.sliderify = function(startmin, startmax, rangemin, rangemax) {
  function sliderToolTip(value) {
    //value|0 - conversion to int
    $(this).html(
      '<span>' + (value|0) + '</span>'
    );
  }

  return $(this).noUiSlider({
    start: [startmin, startmax],
    connect: true,
    step: 1,
    range: {
      'min': rangemin,
      'max': rangemax
    }
  }).Link('upper').to('-inline-<div class="slidertooltip"></div>', sliderToolTip)
  .Link('lower').to('-inline-<div class="slidertooltip"></div>', sliderToolTip);
}

/**
 * Toggle the display of the flight path of the copter.
 */
function toggleFlightPath() {
  $("#map-canvas").copterMap('toggleCopterPath');
}

/**
 * Clear the current displayed flight path of the copter.
 */
function clearFlightPath() {
  $("#map-canvas").copterMap('clearCopterPath');
}

/**
 * Enable waypoints editing.
 */
function waypointsEdit() {
  var pattern = $("#waypoints-pattern option:selected").val();
  $("#map-canvas").copterMap('toggleEditMode', pattern);
  $("#wpt-begin-container").toggleClass('hidden');
  $("#wpt-editalert").toggleClass('hidden');
  $("#waypoint-editor").toggleClass('hidden');
  if (pattern === "exclusion") {
    $("#wpt-exclconfig").toggleClass('hidden');
  }
}


/**
 * Add a new exclusion zone.
 * @return Return_Description
 */
function addExclusionZone() {
  $("#map-canvas").copterMap('addExclusionZone');
}

/**
 * Clear the currently displayed set of waypoints/markers on the map.
 */
function waypointsClear() {
  $("#map-canvas").copterMap('clearActiveMarkers');
}

function detectionShow() {
  var log = $("#detection-review option:selected");
  var data = log.data('detection');
  $("#map-canvas").copterMap('clearDetectionMarkers');
  $.each(data.detected, function(i,v) {
      $("#map-canvas").copterMap('addDetection', v);
  });
  $.each(data.gpslog, function(i, v) {
      $("#map-canvas").copterMap('addDetectionTrack', v);
  });
  
  console.log(data);
}

function detectionExport() {
  var log = $("#detection-review option:selected");
  if (log.val()) {
    window.open("ajax-review-gpx.php?log=" + log.val(), "_blank");
  }
}

function detectionClear() {
  $("#map-canvas").copterMap('clearDetectionMarkers');
}

function jumpTo(dst) {
  $("#map-canvas").copterMap('jumpTo', dst);
}

function updateWptEditor(pattern, coords) {
  $.each(coords, function(index, val) {
    var row = $("<tr/>").append($("<td/>", {text : index+1}));
    var entries = ["lat","lon","alt"];
    for (var i = 0; i < entries.length; i++) {
      var ent = $("<td/>").append($("<input/>", 
        {
          id : "wpt-" + (index+1).toString() + "-" + entries[i],
          type : "number",
          value : val[i],
          class : "form-control"
        }));
      row.append(ent);
    }
    $("#waypoint-editor tbody").append(row);
  });
}

function updateWptInfo(e) {
  var elem = $(e.target);
  var re = /wpt-(\d+)-(.*)/g;
  var res = re.exec(elem.attr("id"));
  if (res) {
    $("#map-canvas").copterMap('updateWaypoint', parseInt(res[1],10)-1, res[2], parseFloat(elem.val()));
  }
}

/**
 * Uses information received from the server to update the displayed parameters.
 * @param [in] ret The JSON object of the CAMERA_STREAM family.
 */
function updateDisplayedCameraConfig(ret) {
  if (ret) {
    switch (ret["THRESH_COLOURSPACE"]) {
      default: case 0:
        $("#camera-thresh-colourspace").val("csp-hsv");
        $("#cal-hue").val([ret["MIN_HUE"], ret["MAX_HUE"]]);
        $("#cal-sat").val([ret["MIN_SAT"], ret["MAX_SAT"]]);
        $("#cal-val").val([ret["MIN_VAL"], ret["MAX_VAL"]]);
        break;
      case 1:
        $("#camera-thresh-colourspace").val("csp-ycbcr");
        $("#cal-y").val([ret["MIN_Y"], ret["MAX_Y"]]);
        $("#cal-cb").val([ret["MIN_Cb"], ret["MAX_Cb"]]);
        $("#cal-cr").val([ret["MIN_Cr"], ret["MAX_Cr"]]);
        break;
    }
    
    $("#camera-thresh-colourspace").trigger("change");
  }
}


/**
 * JS hooks to make the site interactive/display properly.
 */
$(document).ready(function () {
  /* Show the map */
  $("#map-canvas").copterMap();

  /* Enable the tooltips */
  $('[data-toggle="tooltip"]').tooltip()

  /* Prevent the default action of anchor buttons */
  $("a.btn-control").on("click", function (e) {
    e.preventDefault();
  });

  /* Collapse the drop-down menu when clicked */
  $('.navbar-collapse a:not(.dropdown-toggle)').click(function(){
      $(".navbar-collapse ").collapse('hide');
  });

  /*
   * Hide the secondary camera and show the primary camera when in calibration mode.
   * Also leave waypoints edit mode.
   */
  $('#navbar-main a[data-toggle="tab"]').on('shown.bs.tab', function (e) {
    var target = $(e.target).attr("href"); // activated tab
    if (target === "#sidebar-calibration") {
      $('#main-tab-nav a[href="#tab-camera"]').tab('show');
      $("#camera-secondary").hide();
    } else {
      $('#main-tab-nav a[href="#tab-map"]').tab('show');
      $("#camera-secondary").show();
    }
    //Disable edit mode
    $("#map-canvas").copterMap('toggleEditMode', 'off');
    $("#wpt-begin-container").removeClass('hidden');
    $("#wpt-editalert").addClass('hidden');
  });

  /* Retrieve server config when the tab is shown */
  $('#main-tab-nav a[data-toggle="tab"]').on('shown.bs.tab', function (e) {
    var target = $(e.target).attr("href"); // activated tab
    if (target === "#tab-server-config") {
      $("#settings-space").requestSettings();
    }
  });

  /* Reset combo box dropdowns to the first entry on refresh of the page. */
  $("select").each(function () {
    $(this).val($(this).children("option:first").val());
  });
  
  /* Reset checkboxes */
  $(":input").removeAttr("checked");

  /* Show the drawn pattern (if any) on the map */
  $("#waypoints-pattern").change(function() {
    var pattern = $("#waypoints-pattern option:selected").val();
    $("#wpt-spiralconfig").toggleClass("hidden", pattern !== "spiral");
    $("#wpt-exclconfig").toggleClass("hidden", pattern !== "exclusion" || $("#wpt-editalert").hasClass('hidden'));
    $("#map-canvas").copterMap('hideMarkers', "all").copterMap('showMarkers', pattern);
  });
  
  /* Update waypoints on map as the fields are edited */
  $('#waypoint-editor').on('change', 'input', updateWptInfo);
  
  /* Update the fields in the table as the waypoints are moved/deleted */
  $("#map-canvas").on("wptUpdated", function(e) {
    $("#waypoint-editor tbody").empty();
    $("#map-canvas").copterMap('getActiveMarkerCoordinates', updateWptEditor, true);
  });

  /* Update the camera mode */
  $("#camera-mode").change(function() {
    var mode = $("#camera-mode option:selected").val();
    $("#camera-calibration-inputs").addClass('hidden');
    setCameraMode(mode).success(function () {
      if (mode == "999") {
        $("#camera-calibration-inputs").removeClass('hidden');
      }
    });
  });
  
  /* Display the detections on the map. */
  $("#detection-review").change(function() {
    detectionShow();
  });
  
  /* Camera thresholding colourspace menu */
   $("#camera-thresh-colourspace").change(function() {
     console.log("COLOURSPACE UPDATED!");
    var cs = $("#camera-thresh-colourspace option:selected").val();
    $("#cal-hsv").toggleClass("hidden", cs!=="csp-hsv");
    $("#cal-ycbcr").toggleClass("hidden", cs!=="csp-ycbcr");
  });

  /* Camera presets menu */
  $("#camera-colour-presets").change(function() {
    var cs = $("#camera-thresh-colourspace option:selected").val();
    var preset = $("#camera-colour-presets option:selected").val();
    var huePresetMap = {
      "preset-red" : [[-20, 20], [97, 255], [127, 255]],
      "preset-orange" : [[15,36], [97, 255], [127, 255]],
      "preset-yellow" : [[12,45], [62, 255], [153, 255]],
      "preset-green" : [[30,70], [94, 255], [75, 255]],
      "preset-blue" : [[104,152], [83, 255], [70, 255]],
      "preset-white" : [[-180,180], [0, 32], [156, 255]],
      "preset-black" : [[-180,180], [0, 255], [0, 62]]
    };
    
    var ycbcrPresetMap = {
      "preset-red" : [[0, 255], [86, 141], [167, 255]],
      "preset-orange" : [[0, 255], [76, 106], [164, 194]],
      "preset-yellow" : [[0, 255], [0, 91], [118, 159]],
      "preset-green" : [[0, 255], [114, 168], [31, 61]],
      "preset-blue" : [[0, 255], [164, 255], [51, 84]],
      "preset-white" : [[150, 255], [113, 133], [113, 133]],
      "preset-black" : [[0, 105], [113, 133], [113,133]]
    };

    if (cs === "csp-hsv") {
      if (preset in huePresetMap) {
        var vals = huePresetMap[preset];
        $("#cal-hue").val(vals[0]);
        $("#cal-sat").val(vals[1]);
        $("#cal-val").val(vals[2]);
      }
    } else if (cs === "csp-ycbcr") {
      if (preset in ycbcrPresetMap) {
        var vals = ycbcrPresetMap[preset];
        $("#cal-y").val(vals[0]);
        $("#cal-cb").val(vals[1]);
        $("#cal-cr").val(vals[2]);
      }
    }
  });
  
  
  $('#takeoff-modal').on('click', '.pic-doit', function(e) {
    var alt = parseInt($("#takeoff-alt").val(), 10);
    if (alt > 0) {
      takeOff(alt);
    }
  });
  
  $("#mapping-modal-manual").on('click', function(e) {
    var radius = parseInt($("#mapping-alt").val(), 10);
    beginUserMapping(radius);
  });
  
  $("#mapping-modal-auto").on('click', function(e) {
    beginUserMapping();
  });
});
