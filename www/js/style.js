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

function waypointsEdit() {
  var pattern = $("#waypoints-pattern option:selected").val();
  $("#map-canvas").copterMap('toggleEditMode', pattern);
  $("#wpt-begin-container").toggleClass('hidden');
  $("#wpt-editalert").toggleClass('hidden');
  $("#waypoint-editor").toggleClass('hidden');
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
  
  console.log(data);
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

  /* Show the drawn pattern (if any) on the map */
  $("#waypoints-pattern").change(function() {
    var pattern = $("#waypoints-pattern option:selected").val();
    $("#map-canvas").copterMap('hideMarkers', "all").copterMap('showMarkers', pattern);
  });
  
  $('#waypoint-editor').on('change', 'input', updateWptInfo);
  
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

  /* Camera presets menu */
  $("#camera-colour-presets").change(function() {
    var preset = $("#camera-colour-presets option:selected").val();
    var presetMap = {
      "preset-red" : [[-20, 20], [97, 255], [127, 255]],
      "preset-orange" : [[15,36], [97, 255], [127, 255]],
      "preset-yellow" : [[36,51], [97, 255], [127, 255]],
      "preset-green" : [[60,127], [87, 255], [38, 255]],
      "preset-blue" : [[118,182], [97, 255], [127, 255]],
      "preset-white" : [[0,360], [0, 255], [239, 255]],
      "preset-black" : [[-2,2], [0, 7], [0, 15]]
    }

    if (preset in presetMap) {
      var vals = presetMap[preset];
      $("#cal-hue").val(vals[0]);
      $("#cal-sat").val(vals[1]);
      $("#cal-val").val(vals[2]);
    }
  });
});
