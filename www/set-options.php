<!DOCTYPE html>
<html class="light lightpad">
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
	<body class="light">
		<!-- Body -->
		<div id="main-row" class="vstretch light">
			<h1 class="page-header">Settings editor
				<a href="index.php">
					<span class="glyphicon glyphicon-home"></span>
				</a></h1>
			<form id="settings-editor" class="" action="#">
				<div id="settings-space"></div>
				<button type="submit" class="btn btn-default">Submit</button>
			</form>
		</div>
		<script src="js/external/jquery-2.1.1.min.js"></script>
		<script src="js/external/jquery-blink.js"></script>
		<script src="js/external/bootstrap.min.js"></script>
		<script src="js/external/jquery.nouislider.all.min.js"></script>
		<script src="js/ajax.js"></script>
		<script>
			$(document).ready(function () {
				requestSettings();
				$("#settings-editor").submit(function() {
					updateSettings();
					return false;
				});
			});
		</script>
		<!--<script src="js/camera.js"></script>-->	
	</body>
</html>