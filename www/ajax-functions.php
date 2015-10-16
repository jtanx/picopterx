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
        $att = $client->requestAttitude();

        if (isset($source["lat"])) {
          $wp = new \picopter\coordDeg();
          $wp->lat = $source['lat'];
          $wp->lon = $source['lon'];

          $client->updateUserPosition($wp);
        }

        $ans = array('lat' => $coords->lat, 'lon' => $coords->lon, 
               'alt' => $coords->alt, 'bearing' => $client->requestBearing(),
               'roll' => $att->roll, 'pitch' => $att->pitch, 'yaw' => $att->yaw,
               'status' => $client->requestStatus(), 'lidar' => $client->requestLidar());

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
      
      case "beginTakeoff":
        if (isset($source["data"])) {
          $alt = intval($source["data"]);
          $ans = $client->beginTakeoff($alt);
          print "beginTakeoff " . $b[$ans];
          break;
        }
        
      case "beginReturnToLaunch":
        $ans = $client->beginReturnToLaunch();
        print "beginReturnToLaunch " . $b[$ans];
        break;

      case "updateWaypoints":
        if (isset($source["data"])) {
          $waypoints = array();

          foreach ($source["data"] as $i) {
            $wp = new \picopter\coordDeg();
            $wp->lat = $i[0];
            $wp->lon = $i[1];
            $wp->alt = $i[2];
            array_push($waypoints, $wp);
          }

          $ans = $client->updateWaypoints($waypoints);
          print count($waypoints) . " waypoints added.\n";
        } else {
          print "updateWaypoint failed.\n";
        }
        break;
        
      case "updateExclusions":
        if (isset($source["data"])) {
          $zones = array();

          foreach ($source["data"] as $zone) {
            $zlist = array();
            foreach($zone as $i) {
              $wp = new \picopter\coordDeg();
              $wp->lat = $i[0];
              $wp->lon = $i[1];
              $wp->alt = $i[2];
              array_push($zlist, $wp);
            }
            if (sizeof($zlist) > 0) {
              array_push($zones, $zlist);
            }
          }

          $ans = $client->updateExclusions($zones);
          print count($zones) . " exclusion zones added.\n";
        } else {
          $ans = $client->updateExclusions(array());
          print "Clearing exclusions: " . $b[$ans];
        }
        break;
        
      case "updateJoystick":
        if (isset($source["data"]) && sizeof($source["data"]) == 4) {
          $client->updateJoystick($source["data"][0], $source["data"][1],
            $source["data"][2], $source["data"][3]);
          print "OK.\n";
        } else {
          print "updateJoystick failed.\n";
        }
        break;

      case "resetWaypoints":
        $ans = $client->resetWaypoints();
        print "resetWaypoints " . $b[$ans] . "\n";
        break;

      case "beginWaypoints":
        if (isset($source["data"])) {
          $mode = intval($source["data"]);
          $ans = $client->beginWaypointsThread($mode);
          print "beginWaypointsThread " . $b[$ans] . "\n";
        } else {
          print "beginWaypointsThread: No mode specified.\n";
        }
        break;

      case "beginUserTracking":
        $ans = $client->beginUserTrackingThread();
        print "beginUserTrackingThread " . $b[$ans] . "\n";
        break;
        
      case "beginJoystickControl":
        $ans = $client->beginJoystickControl();
        print "beginJoystickControl " . $b[$ans] . "\n";
        break;
        
      case "beginTakingPictures":
        $ans = $client->beginPicturesThread();
        print "beginTakingPictures" . $b[$ans] . "\n";
        break;
        
      case "beginUserMapping":
        if (isset($source["data"])) {
          $radius = intval($source["data"]);
          if ($radius <= 0) {
            $ans = $client->beginUserMappingThread(true, -1);
            print "beginUserMapping " . $b[$ans] . "\n";
          } else {
            $ans = $client->beginUserMappingThread(false, $radius);
            print "beginUserMapping " . $b[$ans] . "\n";
          }
        }
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

      case "requestCameraConfig":
        print $client->requestCameraConfig();
        break;

      case "setCameraConfig":
        if (isset($source["data"])) {
          $client->setCameraConfig($source["data"]);
        }
        break;

      case "doCameraAutoLearning":
        $client->doCameraAutoLearning();
        break;

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
