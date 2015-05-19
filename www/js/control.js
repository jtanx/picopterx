/**
 * Functions to make the web page interactive.  
 */

var canEdit = false;
var canEditMarkers = false;
var canEditBounds = false;
var cameraEnabled = false;
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
  
function cameraMode() {
	if (!cameraEnabled) {
		$("#map-canvas").hide();
		$("#camera-main").show();
		
		url = "http://" + document.domain + ":5000/?action=stream";
		$("#camera-main").html("<img id='camera-main-img' src='" + url + "'/>");
		
		$("#settings-camera").toggleClass('orange-toggle');
	} else {
		$("#camera-main").hide();
		$("#map-canvas").show();
		$("#camera-main").html();
		
		$("#settings-camera").toggleClass('orange-toggle');
	}
	
	cameraEnabled = !cameraEnabled;
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
		
		ajaxSend('updateWaypoints', data);
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