<!DOCTYPE html>
<!-- Redesigned interface (Bootstrap 3 + jQuery) for 2015. -->
<!-- I tried using Node/Bower/Grunt/Yeoman and gave up. -->
<!-- If you're a web developer, sorry. -->
<!-- Primary Author: Jeremy Tan 20933708 -->
<html>
  <head>
    <meta charset="utf-8">
    <title>UWA Copter GCS</title>
    <!-- Needed to scale the contents properly on iOS devices... -->
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <!-- Joystick in IE -->
    <meta http-equiv="X-UA-Compatible" content="IE=Edge">
    <!-- Not LCARS anymore, but eh... -->
    <link rel="icon" href="css/markers/trek.png">
    <!-- Using the Yeti theme from Bootswatch (http://bootswatch.com/yeti/) -->
    <link rel="stylesheet" type="text/css" href="css/external/bootstrap.min.css">
    <link rel="stylesheet" type="text/css" href="css/external/jquery.nouislider.min.css">
    <link rel="stylesheet" type="text/css" href="css/external/leaflet.css">
    <link rel="stylesheet" type="text/css" href="css/external/flightindicators.min.css">
    <link rel="stylesheet" type="text/css" href="css/leaflet-numbered-markers.css">
    <link rel="stylesheet" type="text/css" href="css/style.css">
  </head>

  <body>
    <!-- Navbar -->
    <div class="navbar navbar-default navbar-fixed-top">
      <div class="container">
        <div class="navbar-header">
          <a class="navbar-brand" href="#">
            UWA Copter GCS
          </a>
          <!-- Hamburger -->
          <button class="navbar-toggle" type="button" data-toggle="collapse" data-target="#navbar-main">
            <span class="icon-bar"></span>
            <span class="icon-bar"></span>
            <span class="icon-bar"></span>
          </button>
        </div>
        <div id="navbar-main" class="navbar-collapse collapse">
          <ul class="nav navbar-nav">
            <li class="active"><a aria-expanded="true" href="#sidebar-waypoints" data-toggle="tab">Waypoints</a></li>
            <li><a aria-expanded="false" href="#sidebar-tracking" data-toggle="tab">Tracking</a></li>
            <li><a aria-expanded="false" href="#sidebar-mapping" data-toggle="tab">Mapping</a></li>
            <!--<li><a aria-expanded="false" href="#sidebar-general" data-toggle="tab">General</a></li>-->
            <li class="dropdown">
            <a aria-expanded="false" class="dropdown-toggle" data-toggle="dropdown" href="#">
              Other <span class="caret"></span>
            </a>
              <ul class="dropdown-menu">
                <li><a aria-expanded="false" href="#sidebar-joy" data-toggle="tab">Joystick control</a></li>
                <li><a aria-expanded="false" href="#sidebar-calibration" data-toggle="tab">Camera calibration</a></li>
                <li><a aria-expanded="false" href="#sidebar-data-review" data-toggle="tab">Data review</a></li>
                <li><a aria-expanded="false" href="#sidebar-interface" data-toggle="tab">Interface settings</a></li>
              </ul>
            </li>
          </ul>
        </div>
      </div>
    </div>
    
    <!-- Modals -->
    <!-- Take-off modal -->
    <div class="modal fade" id="takeoff-modal" tabindex="-1" role="dialog" aria-labelledby="takeoff-modal-label">
      <div class="modal-dialog" role="document">
        <div class="modal-content">
          <div class="modal-header">
            <button type="button" class="close" data-dismiss="modal" aria-label="Close"><span aria-hidden="true">&times;</span></button>
            <h4 class="modal-title" id="takeoff-modal-label">Take-off</h4>
          </div>
          <div class="modal-body">
            <div class="form-group">
              <label for="takeoff-alt" class="control-label">Takeoff altitude (m)</label>
              <input class="form-control" id="takeoff-alt" type="number" step="1" min="0" value="5">
            </div>
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-default" data-dismiss="modal">Close</button>
            <button type="button" class="btn btn-primary pic-doit" data-dismiss="modal">Begin</button>
          </div>
        </div>
      </div>
    </div>
    <!-- RTL modal -->
    <div class="modal fade" id="rtl-modal" tabindex="-1" role="dialog" aria-labelledby="rtl-modal-label">
      <div class="modal-dialog" role="document">
        <div class="modal-content">
          <div class="modal-header">
            <button type="button" class="close" data-dismiss="modal" aria-label="Close"><span aria-hidden="true">&times;</span></button>
            <h4 class="modal-title" id="rtl-modal-label">RTL confirmation</h4>
          </div>
          <div class="modal-body">
            Are you sure that you want to perform an RTL?
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-default" data-dismiss="modal">No</button>
            <button type="button" class="btn btn-danger pic-doit" data-dismiss="modal" onclick='returnToLaunch()'>Yes</button>
          </div>
        </div>
      </div>
    </div>
    <!-- Mapping modal -->
    <div class="modal fade" id="mapping-modal" tabindex="-1" role="dialog" aria-labelledby="mapping-modal-label">
      <div class="modal-dialog" role="document">
        <div class="modal-content">
          <div class="modal-header">
            <button type="button" class="close" data-dismiss="modal" aria-label="Close"><span aria-hidden="true">&times;</span></button>
            <h4 class="modal-title" id="mapping-modal-label">Mapping options</h4>
          </div>
          <div class="modal-body">
            What mapping radius do you want to use?
            <div class="form-group">
              <label for="mapping-alt" class="control-label">Manual radius (m)</label>
              <input class="form-control" id="mapping-alt" type="number" step="1" min="0" value="7">
            </div>
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-default" data-dismiss="modal">Cancel</button>
            <button type="button" id="mapping-modal-manual" class="btn btn-primary pic-doit" data-dismiss="modal" onclick='doManualMapping()'>Manual</button>
            <button type="button" id="mapping-modal-auto"  class="btn btn-success pic-doit" data-dismiss="modal" onclick='doLidarMapping()'>Auto (LIDAR)</button>
          </div>
        </div>
      </div>
    </div>

    <!-- Main body content -->
    <div id="main-body" class="container">
      <!-- Tab bar -->
      <ul id="main-tab-nav" class="nav nav-tabs nav-no-underline">
        <li class="dropdown active">
          <a aria-expanded="false" class="dropdown-toggle" data-toggle="dropdown" href="#">
            Display <span class="caret"></span>
          </a>
          <ul class="dropdown-menu">
            <li class="active"><a aria-expanded="true" href="#tab-map" data-toggle="tab">Map</a></li>
            <li><a aria-expanded="false" href="#tab-camera" data-toggle="tab">Camera</a></li>
            <li class="divider"></li>
            <li><a aria-expanded="false" href="#tab-server-config" data-toggle="tab">Server Config</a></li>
          </ul>
        </li>
        <li class=""><a aria-expanded="false" href="#" id="allstop" class="nav-primary btn-control" onclick="allStop()">All Stop</a></li>
        <li class="pull-right visible-xs visible-sm">
          <a aria-expanded="false" href="#right-sidebar-jump">
            <span class="glyphicon glyphicon-chevron-down" aria-hidden="true"></span>
          </a>
        </li>
      </ul>
      <!-- Content below the tab bar -->
      <div class="row">
        <div class="col-md-9">
          <!-- Main tab contents -->
          <div id="main-tab-content" class="tab-content">
            <div class="tab-pane active in semi-fill" id="tab-map">
              <div id="map-canvas" class="fill">
                <!-- Our map area -->
                <div id="copter-hud-primary" class="overlay overlay-tr hidden-xs">
                  <!-- The HUD -->
                  <div class="btn-group">
                    <a class="btn btn-default btn-xs glyphicon glyphicon-dashboard" data-toggle="collapse" aria-expanded="true" href="#copter-hud-items">
                    </a>
                  </div>
                  <div id="copter-hud-items" class="collapse in">
                    <div id="copter-yaw"></div>
                    <div id="copter-att"></div>
                  </div>
                </div>
                <div id="copter-hud-secondary" class="overlay overlay-bl">
                </div>
              </div>
            </div>
            <div class="tab-pane semi-fill text-center" id="tab-camera">
              <div id="camera-primary" class="fill"></div>
            </div>
            <div class="tab-pane semi-fill" id="tab-server-config">
              <form id="settings-editor" class="" action="#">
                <div id="settings-space"></div>
                <div class="form-group">
                  <button type="submit" class="btn btn-default">Submit</button>
                </div>
              </form>
            </div>
            <div class="alert alert-warning" id="status-bar">
              Status unknown.
            </div>
          </div>
          
          <!-- Waypoint editing -->
          <div id="waypoint-editor" class="well hidden">
            <table class="table">
            <thead>
              <tr>
                <th>ID</th>
                <th>Lat</th>
                <th>Lon</th>
                <th>Alt</th>
              </tr>
            </thead>
            <tbody>
            </tbody>
            </table>
            <!-- Hit it with a hammer -->
            <button class="btn btn-default" onclick="$('#waypoint-editor input').trigger('change')">Update</button>
          </div>
        </div>
        
        <!-- Sidebar contents -->
        <div id="right-sidebar-jump" class="nav-jump"></div>
        <div class="col-md-3" id="right-sidebar">
          <div id="sidebar-tab-content" class="tab-content">
            <!-- Waypoints sidebar -->
            <div id="sidebar-waypoints" class="tab-pane active in panel panel-primary">
              <div class="panel-heading">
                <h3 class="panel-title">Waypoints</h3>
              </div>
              <div class="panel-body">
                <div class="form-group">
                  <label>Pattern</label>
                  <select id="waypoints-pattern" class="input-large form-control">
                    <option value="manual" selected="selected">Manual</option>
                    <option value="lawnmower">Lawnmower</option>
                    <option value="spiral">Spiral</option>
                    <option value="exclusion">Exclusion zones</option>
                  </select>
                </div>
                <div class="checkbox hidden" id="wpt-spiralconfig">
                  <label><input type="checkbox" id="wpt-spiraldir"> Face outwards</label>
                </div>
                <div class="btn-group btn-group-justified">
                  <a href="#" class="btn btn-success btn-control hidden" id="wpt-exclconfig" onclick='addExclusionZone()'>Add</a>
                  <a href="#" class="btn btn-default btn-control" onclick='waypointsEdit()'>Edit</a>
                  <a href="#" class="btn btn-default btn-control" onclick='waypointsClear()'>Clear</a>
                </div>
                
                <div class="btn-group btn-block" id="wpt-begin-container">
                  <a href="#" class="btn btn-warning btn-block-dropdown" onclick="beginWaypoints()">Begin Flight</a>
                  <a aria-expanded="false" href="#" class="btn btn-warning dropdown-toggle" data-toggle="dropdown"><span class="caret"></span></a>
                  <ul class="dropdown-menu">
                    <li><a href="#" data-toggle="modal" data-target="#takeoff-modal">Take off</a></li>
                    <li><a href="#" data-toggle="modal" data-target="#rtl-modal">Return to launch (RTL)</a></li>
                  </ul>
                </div>
                <div id="wpt-editalert" class="alert alert-info hidden">
                  Edit mode engaged - click on the map to modify waypoints.
                </div>
              </div>
            </div>
            <!-- Tracking sidebar -->
            <div id="sidebar-tracking" class="tab-pane panel panel-primary">
              <div class="panel-heading">
                <h3 class="panel-title">Tracking</h3>
              </div>
              <div class="panel-body">
                <p>
                  <a href="#" class="btn btn-default btn-block btn-control" id="user-tracker" onclick="beginUserTracking()" data-toggle="tooltip" data-placement="top" title="" data-original-title="When tracking is possible, this button turns blue.">Track User</a>
                </p>
                <p>
                  <a href="#" class="btn btn-default btn-block btn-control" onclick="beginObjectTracking()">Track Object</a>
                </p>
              </div>
            </div>
            <!-- Mapping sidebar -->
            <div id="sidebar-mapping" class="tab-pane panel panel-primary">
              <div class="panel-heading">
                <h3 class="panel-title">Mapping</h3>
              </div>
              <div class="panel-body">
                <p>
                  <a href="#" class="btn btn-default btn-block btn-control" onclick="beginTakingPictures()">Take Pictures</a>
                  <a href="#" class="btn btn-default btn-block btn-control" data-toggle="modal" data-target="#mapping-modal">Begin Mapping</a>
                  <a href="pics/" class="btn btn-success btn-block">Download</a>
                </p>
              </div>
            </div>
            <!-- Joystick control sidebar -->
            <div id="sidebar-joy" class="tab-pane panel panel-primary">
              <div class="panel-heading">
                <h3 class="panel-title">Joystick control</h3>
              </div>
              <div class="panel-body">
              <form id="joy-form">
                <div class="form-group">
                  <label>Using Joystick:</label>
                  <input type="text" id="joy-id" class="form-control" readonly>
                </div>
                <div class="form-group">
                  <label>Dead man's switch</label>
                  <input type="text" id="joy-deadman" class="form-control danger-danger" value="OFF" readonly>
                </div>
                <div class="row">
                  <div class="form-group col-xs-6">
                    <label>Throttle</label>
                    <input type="text" id="joy-throttle" class="form-control" readonly>
                  </div>
                  <div class="form-group col-xs-6">
                    <label>Yaw</label>
                    <input type="text" id="joy-yaw" class="form-control" readonly>
                  </div>
                </div>
                <div class="row">
                  <div class="form-group col-xs-6">
                    <label>X</label>
                    <input type="text" id="joy-x" class="form-control" readonly>
                  </div>
                  <div class="form-group col-xs-6">
                    <label>Y</label>
                    <input type="text" id="joy-y" class="form-control" readonly>
                  </div>
                </div>
                <button type="button" id="joy-begin" class="btn btn-primary btn-block" onclick="gpBegin()">
                Begin
                </button>
                <button type="button" id="joy-end" class="btn btn-warning btn-block hidden" onclick="allStop()">
                End
                </button>
              </form>
              </div>
            </div>
            <!-- Camera calibration sidebar -->
            <div id="sidebar-calibration" class="tab-pane panel panel-primary">
              <div class="panel-heading">
                <h3 class="panel-title">Camera calibration</h3>
              </div>
              <div class="panel-body">
                <div class="form-group">
                  <label>Camera mode</label>
                  <select id="camera-mode" class="input-large form-control">
                    <option value="0" selected="selected">No processing</option>
                    <option value="1">Centre of mass</option>
                    <option value="2">Camshift</option>
                    <option value="3">Connected components</option>
                    <option value="4">Canny glyph detection</option>
                    <option value="5">Thresholding glyph detection</option>
                    <option value="6">Hough circle detection</option>
                    <option value="7">HOG people detection</option>
                    <option value="999">Colour training</option>
                  </select>
                </div>
                <div class="btn-group btn-group-justified">
                  <a href="#" class="btn btn-default btn-control" onclick='toggleLearningThreshold()'>Toggle Threshold</a>
                </div>
                <div id="camera-calibration-inputs" class="hidden">
                  <div class="form-group">
                    <label>Colourspace</label>
                      <select id="camera-thresh-colourspace" class="input-large form-control">
                      <option value="csp-hsv" selected="selected">HSV</option>
                      <option value="csp-ycbcr">YCbCr</option>
                    </select>
                    <label>Presets</label>
                     <select id="camera-colour-presets" class="input-large form-control">
                      <option value="preset-red" selected="selected">Red</option>
                      <option value="preset-orange">Orange</option>
                      <option value="preset-yellow">Yellow</option>
                      <option value="preset-green">Green</option>
                      <option value="preset-blue">Blue</option>
                      <option value="preset-white">White</option>
                      <option value="preset-black">Black</option>
                    </select>
                    <div id="cal-hsv">
                      <label>Hue</label>
                      <div id="cal-hue"></div>
                      <label>Saturation</label>
                      <div id="cal-sat"></div>
                      <label>Value</label>
                      <div id="cal-val"></div>
                    </div>
                    <div id="cal-ycbcr" class="hidden">
                      <label>Y</label>
                      <div id="cal-y"></div>
                      <label>Cb</label>
                      <div id="cal-cb"></div>
                      <label>Cr</label>
                      <div id="cal-cr"></div>
                    </div>
                  </div>
                  <div class="btn-group btn-group-justified">
                    <div class="btn-group">
                      <button type="button" class="btn btn-default" onclick="setCameraLearningSize(true)" data-toggle="tooltip" data-placement="top" title="" data-original-title="Decrease learning size">
                        <span class="glyphicon glyphicon glyphicon-triangle-bottom" aria-hidden="true"></span>
                      </button>
                    </div>
                    <div class="btn-group">
                      <button type="button" class="btn btn-default" onclick="setCameraLearningSize(false)" data-toggle="tooltip" data-placement="top" title="" data-original-title="Increase learning size">
                        <span class="glyphicon glyphicon-triangle-top" aria-hidden="true"></span>
                      </button>
                    </div>
                    <div class="btn-group">
                      <button type="button" class="btn btn-default" onclick="doCameraAutoLearning()" data-toggle="tooltip" data-placement="top" title="" data-original-title="Auto-learn colour thresholds">
                        <span class="glyphicon glyphicon-eye-open" aria-hidden="true"></span>
                      </button>
                    </div>
                    <div class="btn-group">
                      <button type="button" class="btn btn-default" onclick="setCameraLearningValues()" data-toggle="tooltip" data-placement="top" title="" data-original-title="Manually set colour thresholds">
                        <span class="glyphicon glyphicon-ok" aria-hidden="true"></span>
                      </button>
                    </div>
                  </div>
                </div>
              </div>
            </div>
            <!-- Data review sidebar -->
            <div id="sidebar-data-review" class="tab-pane panel panel-primary">
              <div class="panel-heading">
                <h3 class="panel-title">Data review</h3>
              </div>
              <div class="panel-body">
                <p>
                  <label>Waypoints data log</label>
                  <select id="detection-review" class="input-large form-control">
                  </select>
                </p>
                
                <div class="btn-group btn-block">
                  <a href="#" class="btn btn-primary btn-block-dropdown" onclick='detectionShow()'>Show</a>
                  <a href="#" class="btn btn-primary dropdown-toggle" data-toggle="dropdown"><span class="caret"></span></a>
                  <ul class="dropdown-menu">
                    <li><a href="#" onclick='detectionExport()'>Export (GPX)</a></li>
                    <li><a href="#" onclick='detectionClear()'>Clear</a></li>
                    <li class="divider"></li>
                    <li><a href="#" onclick='getDetectedObjects()'>Refresh</a></li>
                  </ul>
                </div>
                <a href="videos" class="btn btn-default btn-block btn-control">Download video</a>
                <a href="pics" class="btn btn-default btn-block btn-control">Download pictures</a>
              </div>
            </div>
            <!--
            <div id="sidebar-general" class="tab-pane panel panel-primary">
              <div class="panel-heading">
                <h3 class="panel-title">General copter control</h3>
              </div>
              <div class="panel-body">
                <div class="btn-group btn-block">
                  <a href="#" class="btn btn-warning btn-block-dropdown" data-toggle="modal" data-target="#takeoff-modal">Take off</a>
                  <a aria-expanded="false" href="#" class="btn btn-warning dropdown-toggle" data-toggle="dropdown"><span class="caret"></span></a>
                  <ul class="dropdown-menu">
                    <li><a href="#" data-toggle="modal" data-target="#rtl-modal">Return to launch (RTL)</a></li>
                  </ul>
                </div>
              </div>
            </div>
            -->
            <!-- Interface settings sidebar -->
            <div id="sidebar-interface" class="tab-pane panel panel-primary">
              <div class="panel-heading">
                <h3 class="panel-title">Interface settings</h3>
              </div>
              <div class="panel-body">
                <p>
                  <a href="#" class="btn btn-default btn-block btn-control" onclick='toggleFlightPath()'>Toggle Flight Path</a>
                  <a href="#" class="btn btn-default btn-block btn-control" onclick='clearFlightPath()'>Clear Flight Path</a>
                </p>
                <p>Jump to:</p>
                <div class="btn-group btn-group-justified">
                  <a href="#" class="btn btn-default btn-control" onclick='jumpTo("copter")'>Copter</a>
                  <a href="#" class="btn btn-default btn-control" onclick='jumpTo("user")'>User</a>
                </div>
              </div>
            </div>
          </div>
          <!-- Mini camera view -->
          <div id="camera-secondary" class="well well-sm"></div>
          <!-- I don't know why this div needs to be here, but it helps with padding on IE -->
          <div class="nop"></div>
        </div>
      </div>
    </div>

    <!-- Include JavaScript at the end for speed -->
    <script src="js/external/jquery-2.1.4.min.js"></script>
    <script src="js/external/jquery.nouislider.all.min.js"></script>
    <script src="js/external/jquery.flightindicators.min.js"></script>
    <script src="js/external/bootstrap.min.js"></script>
    <script src="js/external/leaflet.js"></script>
    <script src="js/jquery.coptermap.js"></script>
    <script src="js/ajax.js"></script>
    <script src="js/joystick.js"></script>
    <script src="js/style.js"></script>
    <script src="js/autorun.js"></script>
  </body>
</html>
