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
		<div id="main-row" class="row vstretch">
			<div id="mainwindow" class="col-md-7 vstretch">
				<img id="camera-secondary" class="vstretch" src="#">
			</div>
			<div id="sidepanel" class="col-md-5">
				<div id="sidepanel-top" class="row">
					<!-- Sidepanel - buttons on the left -->
					<div id="menu-button-holder" class="col-md-3 col-sm-2 col-xs-4">
						<button type="button" class="cbtn nav-a" onclick="allStop()">All Stop</button>
						<a href="index.php" class="cbtn nav-d" onclick="">Go back</a>
						<button type="button" class="cbtn nav-d" onclick="window.close()">Close</button>
					</div>
					
					<!-- Sidepanel - information box on the right -->
					<div id="secondary-button-holder" class="col-md-9 col-sm-10 col-xs-8">
						<div id="menu-top-a">
							<div id="menu-top-b"></div>
						</div>
						
						<div id="settings-holder">
							<div class="headline">
								<span class="h3">Camera stream</span>
							</div>
						</div>
					</div>
				</div><!-- row -->
				
				<div id="sidepanel-top-closeout" class="row">
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
			</div>
		</div>
		<script src="js/external/jquery-2.1.1.min.js"></script>
		<script src="js/external/jquery-blink.js"></script>
		<script src="js/external/bootstrap.min.js"></script>
		<script src="js/external/jquery.nouislider.all.min.js"></script>
		<script src="js/ajax.js"></script>
		<script>
			$(document).ready(function () {
				var url = "http://" + document.domain + ":5000/?action=stream";
				$("#camera-secondary").attr("src", url);
			});
		</script>
		<!--<script src="js/camera.js"></script>-->	
	</body>
</html>