<?php
	include('ajax-default-options.php');
	
	$b = Array(false => 'false', true => 'true');

//	print "Received '" . $_POST['action'] . "'<p>";
	
	//Yeah, because we care about CSRF...
	$action  = $_POST["action"];
	$source = $_POST;
	if (!isset($action)) {
		$action = $_GET["action"];
		$source = $_GET;
	}
	
	
	if (isset($action)) {
		switch ($action) {
			
			case "requestAll":
				$coords = $client->requestCoords();
				
				if (isset($source["lat"])) {
					$wp = new \picopter\coordDeg();
					$wp->lat = $source['lat'];
					$wp->lon = $source['lon'];
					
					$client->updateUserPosition($wp);
				}
				
				$ans = array('lat' => $coords->lat, 'lon' => $coords->lon,
							 'bearing' => $client->requestBearing(),
							 'status' => $client->requestStatus());
				
				echo json_encode($ans);
				break;
				
			case "requestCoords":
				$ans = $client->requestCoords();
				print $ans->lat . "," . $ans->lon;
				break;
				
			case "requestBearing":
				$ans = $client->requestBearing();
				print "Bearing: " . $ans . " degrees.\n";
				break;
				
			case "requestStatus":
				$ans = $client->requestStatus();
				print $ans . "\n";
				break;
				
			case "requestSettings":
				//A serialised string wrapped in a protocol buffer!~
				//Probably not a very good way to use thrift, but whatever.
				$current = json_decode($client->requestSettings());
				$output = $defaultOptions;
				//Loop through each option family, e.g. 'CAMERA_STREAM'
				foreach ($current as $k => $v) {
					//Loop through each option, e.g. 'PROCESS_WIDTH'
					foreach ($v as $ko => $kv) {
						$output[$k][$ko] = $kv;
					}
				}
				print json_encode($output) . "\n";
				break;
				
			case "updateSettings":
				$opts = json_decode("{}");
				$ret = false;
				
				if (isset($source["data"])) {
					$iopts = filterOptions(json_decode($source["data"]));
					if ($iopts) {
						$ret = $client->updateSettings(json_encode($iopts));
					}
				}
				
				print $b[$ret] . "\n";
				break;
			
			case "allStop":
				$ans = $client->allStop();
				print "allStop " . $b[$ans];
				break;
				
			case "updateWaypoints":
				if (isset($source["data"])) {
					$waypoints = array();
					
					foreach ($source["data"] as $i) {
						$wp = new \picopter\coordDeg();
						$wp->lat = $i[0];
						$wp->lon = $i[1];
						array_push($waypoints, $wp);
					}
					
					$ans = $client->updateWaypoints($waypoints);
					print count($waypoints) . " waypoints added.\n";
				} else {
					print "updateWaypoint failed.\n";
				}
				break;
				
			case "resetWaypoints":
				$ans = $client->resetWaypoints();
				print "resetWaypoints " . $b[$ans] . "\n";
				break;
			
			case "beginManual":
				$ans = $client->beginWaypointsThread();
				print "beginWaypointsThread " . $b[$ans] . "\n";
				break;
				
			case "beginAuto":
				$ans = $client->beginLawnmowerThread();
				print "beginLawnmowerThread " . $b[$ans] . "\n";
				break;
				
			case "beginUserTracking":
				$ans = $client->beginUserTrackingThread();
				print "beginUserTrackingThread " . $b[$ans] . "\n";
				break;
				
			case "beginObjectTracking":
				if (isset($source["data"])) {
					$method = intval($source["data"]);
					$ans = $client->beginObjectTrackingThread($method);
				} else {
					//Default to strafing tracking
					$ans = $client->beginObjectTrackingThread(0);
				}
				print "beginObjectTracking " . $b[$ans] . "\n";
				break;

			case "setCameraMode":
				if (isset($source["data"])) {
					$mode = intval($source["data"]);
					$ans = $client->setCameraMode($mode);
					print $ans . "\n";
				}
				break;

			case "requestCameraMode":
				print $client->requestCameraMode() . "\n";
				break;

			case "setCameraLearningSize":
				if (isset($source["data"])) {
					$decrease = intval($source["data"]) != 0;
					$client->setCameraLearningSize($decrease);
				}
				break;

			case "showLearningThreshold":
				if (isset($source["data"])) {
					$show = intval($source["data"]) != 0;
					$client->showLearningThreshold($show);
				}
				break;

			case "doCameraAutoLearning":
				$ans = $client->doCameraAutoLearning();
				echo json_encode($ans);
				break;

			case "setCameraLearningValues":
				$ans = $client->setCameraLearningValues($source["data"]);
				echo json_encode($ans);
				break;

			case "requestLearningHue":
				print $client->requestLearningHue();
				
			case "requestNextWaypoint":
				$ans = $client->requestNextWaypoint();
				print $ans-lat . ", " . $ans->lon;
				break;

			default:
				echo "Invalid request: '" . $source['action'] . "'";
		}
	} else {
		echo "No request specified";
	}
?>
