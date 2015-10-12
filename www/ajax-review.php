<?php
  function parseDetected($file) {
    $fp = fopen($file, "r");
    $entries = array();
    $entry = array();
    $gpslog = array();
    $ret = array();
    
    while (($buffer = fgets($fp)) !== false) {
        if (preg_match("/(\d{2})\/(\d{2})\/(\d{4}) (\d{2}):(\d{2}):(\d{2}):\s*Detected object: ID: (\d+)/", $buffer, $matches)) {
          if ($entry) {
            array_push($entries, $entry);
            $entry = array();
          }
          
          $date = new DateTime();
          $date->setTimezone(new DateTimeZone('Australia/Perth'));
          $date->setDate(intval($matches[3]), intval($matches[2]), intval($matches[1]));
          $date->setTime(intval($matches[4]), intval($matches[5]), intval($matches[6]));
          
          $entry["id"] = intval($matches[7]);
          $entry["timestamp"] = $date->format(DateTime::ISO8601);
        } else if (preg_match("/Location: \(([^,]+), ([^,]+), ([^)]+)\) \[([^\]]+)\]/", $buffer, $matches)) {
          $entry["lat"] = floatval($matches[1]);
          $entry["lon"] = floatval($matches[2]);
          $entry["alt"] = floatval($matches[3]);
        } else if (preg_match("/Image: .*(wpt_.*)/", $buffer, $matches)) {
          $entry["image"] = "pics/".$matches[1];
        } else if (preg_match("/(\d{2})\/(\d{2})\/(\d{4}) (\d{2}):(\d{2}):(\d{2}):\s*At: \(([^,]+), ([^,]+), ([^\)]+)\) \[([^\]]+)\]/", $buffer, $matches)) {
          $ent = array();
          $date = new DateTime();
          $date->setTimezone(new DateTimeZone('Australia/Perth'));
          $date->setDate(intval($matches[3]), intval($matches[2]), intval($matches[1]));
          $date->setTime(intval($matches[4]), intval($matches[5]), intval($matches[6]));
          
          $ent["time"] = $date->format(DateTime::ISO8601);
          $ent["lat"] = floatval($matches[7]);
          $ent["lon"] = floatval($matches[8]);
          $ent["alt"] = floatval($matches[9]);
          $ent["heading"] = floatval($matches[10]);
          array_push($gpslog, $ent);
        }
    }
    fclose($fp);
    
    if ($entry) {
      array_push($entries, $entry);
    }
    
    array_push($ret, $entries);
    array_push($ret, $gpslog);
    return $ret;
  }

  function scanLogs($dir) {
    $files = scandir($dir);
    $entries = array();
    
    foreach ($files  as $value) {
      $matches = array();
      if (preg_match("/^waypoints-(\d{4})-(\d{2})-(\d{2})-(\d+)\.txt$/", $value, $matches)) {
        $entry = array();
        $date = new DateTime();
        $date->setTimezone(new DateTimeZone('Australia/Perth'));
        $date->setDate(intval($matches[1]), intval($matches[2]), intval($matches[3]));
        $entry["log"] = $value;
        $entry["timestamp"] = $date->format(DateTime::ISO8601);
        
        $data = parseDetected($dir . "/" . $value);
        $entry["detected"] = $data[0];
        $entry["gpslog"] = $data[1];
        
        array_push($entries, $entry);
      }
    }
    
    print json_encode($entries);
  }
  
  scanLogs("/home/pi/logs");
?>
