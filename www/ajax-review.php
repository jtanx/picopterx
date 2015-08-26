<?php
  function parseDetected($file) {
    $fp = fopen($file, "r");
    $entries = array();
    $entry = array();
    
    while (($buffer = fgets($fp)) !== false) {
        if (preg_match("/(\d{2})\/(\d{2})\/(\d{4}) (\d{2}):(\d{2}):(\d{2})\s*Detected object: ID: (\d+)/", $buffer, $matches)) {
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
        } else if (preg_match("/Location: \(([^,]+), ([^)]+)\) at ([^m]+)m/", $buffer, $matches)) {
          $entry["lat"] = floatval($matches[1]);
          $entry["lon"] = floatval($matches[2]);
          $entry["alt"] = floatval($matches[3]);
        } else if (preg_match("/Image: .*(wpt_.*)/", $buffer, $matches)) {
          $entry["image"] = "pics/".$matches[1];
        }
    }
    fclose($fp);
    
    if ($entry) {
      array_push($entries, $entry);
    }
    return $entries;
  }

  function scanLogs($dir) {
    $files = scandir($dir);
    $entries = array();
    
    foreach ($files  as $value) {
      $matches = array();
      if (preg_match("/^waypoints-(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})(\d{2})\.txt$/", $value, $matches)) {
        $entry = array();
        $date = new DateTime();
        $date->setTimezone(new DateTimeZone('Australia/Perth'));
        $date->setDate(intval($matches[1]), intval($matches[2]), intval($matches[3]));
        $date->setTime(intval($matches[4]), intval($matches[5]), intval($matches[6]));
        $entry["log"] = $value;
        $entry["timestamp"] = $date->format(DateTime::ISO8601);
        $entry["detected"] = parseDetected($dir . "/" . $value);
        
        array_push($entries, $entry);
      }
    }
    
    print json_encode($entries);
  }
  
  scanLogs("/home/pi/logs");
?>
