/**
 * Functions to make the web page interactive.  
 */

var canEdit = false;
var canEditMarkers = false;
var canEditBounds = false;
var pathEnabled = true;

/**
 *  Make a div into a slider
 *  @param [in] selector The jQuery selector
 *  @param [in] startmin Minimum starting value
 *  @param [in] startmax Maximum starting value
 *  @param [in] rangemin The range minimum
 *  @param [in] rangemax The range maximum
 *  @return The object
 */
function sliderify(selector, startmin, startmax, rangemin, rangemax) {
  function sliderToolTip(value) {
    //value|0 - conversion to int
    $(this).html(
      '<span>' + (value|0) + '</span>'
    );
  }
  
  return $(selector).noUiSlider({
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
 * Toggles what is displayed in the main viewport, depending on the given selector. 
 */
function toggleMain(selector, contents) {
  if (selector && !$(selector).is(":visible")) {
    $("#mainwindow > div").not(selector).hide();
    $(selector).show().append(contents);
  } else {
    $("#mainwindow > div").not("#map-canvas").hide();
    $("#map-canvas").show();
    $(selector).empty();
  }
  
  if (!$("#camera-main").is(":visible")) {
    var url = "http://" + document.domain + ":5000/?action=stream";
    $("#camera-secondary").show();
    $("#camera-secondary").html("<img id='camera-secondary-img' src='" + url + "'/>");
  } else {
    $("#camera-secondary").hide();
    $("#camera-secondary").empty();
  }
}

/**
 * Toggles the display of the camera in the main viewport. 
 */
function toggleCamera() {
  var url = "http://" + document.domain + ":5000/?action=stream";

  toggleMain("#camera-main", $("<img/>", {id: "camera-main-img", "class":"vstretch", src:url}));
  $("#settings-camera").toggleClass('orange-toggle');
  $("#settings-opts").removeClass('orange-toggle');
}

/**
 * Toggles the display of the settings editor in the main viewport. 
 */
function toggleOpts() {
  toggleMain("#opts-main", $("<iframe/>", {src : "set-options.php", style : "width:100%;height:100%;"}));
  $("#settings-opts").toggleClass('orange-toggle');
  $("#settings-camera").removeClass('orange-toggle');
}

function setTab(tab) {
  tabs = ["manual-holder", "auto-holder", "tracking-holder", "status-holder", "calibration-holder", "settings-holder"];
  
  $.each(tabs, function(i, val) {
    if (val == tab) {
      $("#" + val).show();
    } else {
      $("#" + val).hide();
    }
  });
}

function manualMode() {
  if (!canEdit) {
    
    setTab("manual-holder");
    
    showMarkers(markers);
    hideMarkers(bounds);
    hideRectangle();
  }
}

function autoMode() {
  if (!canEdit) {
    setTab("auto-holder");
    hideMarkers(markers);
    showMarkers(bounds);
    showRectangle();

  }
}

function trackingMode() {
  if (!canEdit) {
    setTab("tracking-holder");
    hideMarkers(markers);
    hideMarkers(bounds);
    hideRectangle();
  }
}

function statusMode() {
  if (!canEdit) {
    setTab("status-holder");
  }
}

function calibrationMode() {
  if (!canEdit) {
    setTab("calibration-holder");
  }
}

function settingsMode() {
  if (!canEdit) {
    setTab("settings-holder");
  }
}

function togglePath() {
  if (!pathEnabled) {
    addPath();
    $("#settings-path").toggleClass('orange-toggle');
  } else {
    removePath();
    $("#settings-path").toggleClass('orange-toggle');
  }
  
  pathEnabled = !pathEnabled;
}

function toggleEdit(data) {
  $("#manual-edit").toggleClass('btn-pressed');
  $("#auto-edit").toggleClass('btn-pressed');
  
  $("#mainwindow").toggleClass('orange-toggle');
  
  $("button[class*='nav-']").not('.nav-a').toggleClass('btn-disabled');
  $("button[id*='begin']").toggleClass('btn-disabled');
  
  canEdit = !canEdit;
  
  if (canEdit) {
    $("#information").html('<span id="blinkbox" style="background-color: #FB8500;">&nbsp;&nbsp;&nbsp;&nbsp;</span> Edit mode engaged. Use the map.');
    $("#blinkbox").blink({delay:1000});
    
    $.each( data, function( index, value ){
      value.dragging.enable();
    });
  } else {
    $("#blinkbox").unblink();
    $("#information").html('');
    
    $.each( data, function( index, value ){
      value.dragging.disable();
    });
    
    ajaxSend('updateWaypoints', packageCoordinates(data));
  }
}

function toggleMarkersEdit() {
  toggleEdit(markers);
  canEditMarkers = !canEditMarkers;
  
  toggleMarkerRed(markers);
}

function toggleBoundsEdit() {
  toggleEdit(bounds);
  canEditBounds = !canEditBounds;

  toggleMarkerRed(bounds);
}