<?php
  //Convert the text log to a GPX log, zipped with the pictures.
  $log  = $_POST["log"];
  $source = $_POST;
  if (!isset($log)) {
    $log = $_GET["log"];
    $source = $_GET;
  }
  
  if (isset($log) && preg_match("/^waypoints-(\d{4})-(\d{2})-(\d{2})-(\d+)\.txt$/", $log, $m)) {
    $filename = "waypoints-" . $m[1] . "-" . $m[2] . "-" . $m[3] . "-" . $m[4] . ".zip";
    header('Content-type: application/zip');
    header("Content-disposition: attachment; filename=\"" . $filename . "\"");
    ob_clean();
    flush();
    
    $zip = new ZipArchive();
    $filename = "/home/pi/logs/" . $filename;
    if (file_exists($filename)) {
      readfile($filename);
    } else if ($zip->open($filename, ZipArchive::CREATE) === TRUE) {
      $cmd = "python waypoints_parser.py /home/pi/logs/" . $log;
      //Completely safe...
      exec($cmd, $gpx);
      $gpx = implode("\n", $gpx);
      $zip->addFromString(substr($log, 0, -4) . ".gpx", $gpx);
      $zip->addFile("/home/pi/logs/" . $log, $log);
      
      $zip->addEmptyDir("pics");
      $templ = "wpt_" . $m[1] . "-" . $m[2] . "-" . $m[3] . "-" . $m[4];
      $pics = scandir("/home/pi/pics");
      foreach ($pics as $value) {
        if (preg_match("/^". $templ . "/", $value)) {
          $zip->addFile("/home/pi/pics/" . $value, "pics/" . $value);
        }
      }
      $zip->close();
      
      readfile($filename);
    }
  }
?>