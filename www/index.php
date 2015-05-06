<!DOCTYPE html>
<html>
	<head>
		<!-- Header. CSS includes. -->
		<meta charset="utf-8">
		<title>Copter LCARS</title>
		<link rel="icon" href="css/markers/trek.png" />
		<meta name="viewport" content="width=device-width, initial-scale=1.0">
		<link rel="stylesheet" type="text/css" href="css/bootstrap.min.css">
		<link rel="stylesheet" type="text/css" href="css/bootstrap-theme.min.css">
		<link rel="stylesheet" type="text/css" href="css/jquery.nouislider.min.css">
		<link rel="stylesheet" type="text/css" href="css/style.css">
		<link rel="stylesheet" href="css/leaflet.css" />
		<link rel="stylesheet" href="css/leaflet-numbered-markers.css" />
		
	</head>
	<body>
		<!-- Body -->
		<div class="row vstretch">
			<div id="mainwindow" class="col-md-7">
				<div id="map-canvas"></div>
				<!--<div id="camera-main" style="display:none;"></div>-->
			</div>
			<div id="sidepanel" class="col-md-5">
				<div class="row">
					<div id="menu-button-holder" class="col-md-3 col-sm-2 col-xs-4">
						<button type="button" class="cbtn nav-a" onclick="allStop()">All Stop</button>
						<button type="button" class="cbtn nav-b" onclick="statusMode()">Status</button>
						<button type="button" class="cbtn nav-c" onclick="manualMode()">Manual</button>
						<button type="button" class="cbtn nav-c" onclick="autoMode()">Automatic</button>
						<button type="button" class="cbtn nav-c" onclick="trackingMode()">Tracking</button>
						<button type="button" class="cbtn nav-c" onclick="calibrationMode()">Calibration</button>
						<button type="button" class="cbtn nav-d" onclick="settingsMode()">Settings</button>
					</div>
					<div id="secondary-button-holder" class="col-md-9 col-sm-10 col-xs-8">
						<div id="menu-top-a">
							<div id="menu-top-b"></div>
						</div>
						
						<div id="settings-holder" style="display: none">
							<div class="headline">
								<span class="h3">Settings</span>
							</div>
							
							<button id="settings-camera" class="cbtn roundbtn" onclick="cameraMode();">Toggle Camera</button>
							<button id="settings-path" class="cbtn roundbtn" onclick="togglePath()">Toggle Flight Path</button>
						</div>
						
						<div id="status-holder" style="display: none">
							<div class="headline">
								<span class="h3">Status Report</span>
							</div>
							
							<div id="bearing" class="text"></div>
							<div id="response" class="text"></div>
						</div>
						
						<div id="manual-holder" >
							<div class="headline">
								<span class="h3">Manual Mode</span>
							</div>
							
							<button id="manual-edit" class="cbtn roundbtn" onclick="toggleMarkersEdit();">Edit Waypoints</button>
							<button id="manual-reset" class="cbtn roundbtn" onclick="clearMarkers(true)">Reset Waypoints</button>
							<button id="manual-begin" class="cbtn squarebtn" onclick="beginManual()">Begin Flight</button>
						</div>
						
						<div id="auto-holder" style="display: none">
							<div class="headline">
								<span class="h3">Automatic Mode</span>
							</div>

							<button id="auto-edit" class="cbtn roundbtn" onclick="toggleBoundsEdit()">Edit Boundaries</button>
							<button id="auto-reset" class="cbtn roundbtn" onclick="clearMarkers(true)">Reset Boundaries</button>
							<button id="auto-begin" class="cbtn squarebtn" onclick="beginAuto()">Begin Flight</button>
						</div>
						
						<div id="tracking-holder" style="display: none">
							<div class="headline">
								<span class="h3">Tracking Mode</span>
							</div>

							<button id="track-user" class="cbtn roundbtn" onclick="beginUserTracking()">Track Device</button>
							<button id="track-object" class="cbtn roundbtn disabled" onclick="beginObjectTracking(0)">Track Strafe</button>
							<button id="track-rotate" class="cbtn roundbtn disabled" onclick="beginObjectTracking(1)">Track Rotate</button>
						</div>
						
						<div id="calibration-holder" style="display: none">
							<div class="headline">
								<span class="h3">Camera Calibration</span>
							</div>
							<button id="cycle-mode" class="cbtn roundbtn disabled" onclick="setCameraMode()">Cycle mode</button>
							<button id="cal-learn" class="cbtn roundbtn disabled" onclick="doCameraAutoLearning()">Auto learn</button>
							
							<div class="row boxtext">
								<div class="col-md-5">
									<div class="text">Hue</div>
									<div id="cal-hue"></div>
									<div class="text">Saturation</div>
									<div id="cal-sat"></div>
									<div class="text">Value</div>
									<div id="cal-val"></div>
								</div>
								<div class="col-md-7">
									<button id="cal-inc" class="cbtn roundbtn disabled" onclick="setCameraLearningSize(false)">Inc size</button>
									<button id="cal-dec" class="cbtn roundbtn disabled" onclick="setCameraLearningSize(true)">Dec size</button>
									<button id="cal-man" class="cbtn roundbtn disabled" onclick="setCameraLearningValues()">Man. set</button>
								</div>
							</div>
						</div>
						
						<div id="information" class="text"></div>
						<div id="status" class="text"></div>
					</div>
				</div><!-- row -->
				
				<div id="tertiary-holder">
					<div class="row">
						<div class="col-md-3 col-sm-2 hidden-xs" style="padding-right: 0;">
							<div id="tertiary-menu-a">
							</div>
						</div>
						<div class="col-md-9 col-sm-10 hidden-xs" style="padding-left: 0;">
							<div id="tertiary-menu-b">
								<div id="menu-top-c">
									<div id="menu-top-d"></div>
								</div>
							</div>
						</div>
					</div>
					
					<div class="row">
						<div class="col-md-1 col-sm-1 hidden-xs" style="padding-right: 0;">
							<div id="tertiary-menu-c">
							</div>
						</div>
						<div class="col-md-11 col-sm-11" style="padding-left: 0;">
							<div id="tertiary-menu-d">
								<div id="menu-bottom-a">
									<div id="menu-bottom-b"></div>
								</div>
								<div class="headline">
									<span class="h3">Camera Feed</span>
								</div>
								<div class="boxtext">
									<div id="camera-mode" class="text col-md-8 col-sm-8">Connected components</div>
									<button type="button" class="col-md-4 col-sm-4" onclick="toggleLearningThreshold()">Toggle threshold</button>
								</div>
								
								<div id="camera-secondary"></div>
							</div>
						</div>
					</div>
				</div>
				
			</div>
		</div>
		
		<script src="/js/external/jquery-2.1.1.min.js"></script>
		<script src="/js/external/jquery-blink.js"></script>
		<script src="/js/external/bootstrap.min.js"></script>
		<script src="/js/external/jquery.nouislider.all.min.js"></script>
		<script src="/js/external/leaflet.js"></script>
		<script src="/js/leaflet-numbered-markers.js"></script>
		<script src="/js/map.js"></script>
		<script src="/js/ajax.js"></script>
		<script src="/js/control.js"></script>
		<!--<script src="/js/camera.js"></script>-->	
	</body>
</html>




<!-- Footer. Javascript includes. 
		<script src="/jwplayer/jwplayer.js"></script>
		<script src="/js/piCameraStream.js"></script>
		
		<div id="sidepanel_bottom">
				<object type="application/x-shockwave-flash" data="jwplayer/jwplayer.flash.swf" width="100%" height="100%" bgcolor="#000000" id="video-jwplayer" name="video-jwplayer" tabindex="0">
					<param name="allowfullscreen" value="true">
					<param name="allowscriptaccess" value="always">
					<param name="seamlesstabbing" value="true">
					<param name="wmode" value="opaque">
				</object>
				<div id="video-jwplayer_aspect" style="display: none;"></div>
				<div id="video-jwplayer_jwpsrv" style="position: absolute; top: 0px; z-index: 10;"></div>
			</div>-->		
