/**
 * @file joystick.js
 * @brief Joystick control routines. Because why not.
 */

/** Handle to our gamepad. **/
var gamepad;
/** Indicates if we should send joystick data to server. **/
var sendData = false;

/** The mapping we use. **/
var mapping = {
  //Button mappings
  ALLSTOP : 5,
  DEADMAN : 4,
  /*
  ALT_F: 12,
  ALT_B: 13,
  ALT_L: 14,
  ALT_R: 15,
  */
  //Axis mappings
  THROTTLE: 1,
  YAW : 0,
  Y: 3,
  X: 2
};

/**
 * Because of JavaScript closures...
 * @return The current gamepad instance.
 */
function getGamepad() {
  return gamepad;
}

/**
 * Fallback poller for chrome.
 */
function gpAvailPoller() {
  var gp = navigator.webkitGetGamepads();
  if (gp.length > 0 && typeof gamepad === "undefined") {
    gamepad = gp[0];
    $("#joy-id").val(gamepad.id);
  } else if (gp.length == 0) {
    gamepad = undefined;
    $("#joy-id").val("No joypad connected");
  }
  setTimeout(gpAvailPoller, 500);
}

/**
 * Initialises the gamepad/joystick functionality.
 */
function initGamepads() {
  /* Reset the display */
  $("#joy-form")[0].reset();
  
  $("#allstop").on("allstop", function () {
    sendData = false;
    $("#joy-end").addClass("hidden");
    $("#joy-begin").removeClass("hidden");
  });
  
  if (navigator.getGamepads || navigator.webkitGetGamepads) {
    if (navigator.webkitGetGamepads) {
      gpAvailPoller();
    } else {
      function gamepadHandler(event, connecting) {
        var gp = event.gamepad;
        
        if (connecting && typeof gamepad === "undefined") {
          gamepad = gp;
          $("#joy-id").val(gamepad.id);
        } else if (gp.id === gamepad.id) {
          var gps = navigator.getGamepads();
          var ng;
          for (var i = 0; i < gps.length; i++) {
            if (gps[i].id !== gamepad.id) {
              ng = gps[i];
              break;
            }
          }
          if (typeof ng !== "undefined") { 
            gamepad = ng;
            $("#joy-id").val(gamepad.id);
          } else {
            gamepad = undefined;
            $("#joy-id").val("No joypad connected");
          }
        }
      }

      window.addEventListener("gamepadconnected", 
        function(e) { gamepadHandler(e, true); }, false);
      window.addEventListener("gamepaddisconnected",
        function(e) { gamepadHandler(e, false); }, false);
      
    }
  } else {
    $("#joy-id").val("Unsupported. :(");
  }
}

/**
 * Apply deadzoning to the parameter.
 * @param [in] val The value to deadzone.
 * @param [in] lims The positive upper deadzone limit.
 * @return The deadzoned value.
 */
function deadzone(val, lims) {
  if (val > -lims && val < lims) { //Deadzoning
    val = 0;
  } else if (val > 0) { //Scaling
    val = (val-lims)/(1-lims);
  } else { //Scaling
    val = (val+lims)/(1-lims);
  }
  return val;
}

/**
 * Gamepad API requires polling the state for current status.
 */
function gpPoller() {
  var gp = getGamepad();
  var gogogo = false;
  
  if (typeof gamepad !== "undefined") {
    $("#joy-deadman").toggleClass("danger-danger", !gp.buttons[mapping.DEADMAN].pressed);
    $("#joy-deadman").toggleClass("success-success", gp.buttons[mapping.DEADMAN].pressed);
    
    if (gp.buttons[mapping.ALLSTOP].pressed) {
      allStop();
      console.log("STOP!");
    } else if (gp.buttons[mapping.DEADMAN].pressed) {
      $("#joy-deadman").val("ON");
      gogogo = true;
    } else {
      $("#joy-deadman").val("OFF");
    }
    
    var throttle = (deadzone(-gp.axes[mapping.THROTTLE], 0.4) * 100).toFixed(0);
    var yaw = (deadzone(gp.axes[mapping.YAW], 0.3) * 100).toFixed(0);
    var x = (deadzone(gp.axes[mapping.X], 0.2)  * 100).toFixed(0);
    var y = (deadzone(-gp.axes[mapping.Y], 0.2)  * 100).toFixed(0);
    
    $("#joy-throttle").val(throttle);
    $("#joy-yaw").val(yaw);
    $("#joy-x").val(x);
    $("#joy-y").val(y);
    
    if (gogogo && sendData) {
      var pack = [throttle,yaw, x, y];
      ajaxSend("updateJoystick", pack);
    }
    
    setTimeout(gpPoller, 200);
  } else {
    $("#joy-throttle").val(0);
    $("#joy-yaw").val(0);
    $("#joy-x").val(0);
    $("#joy-y").val(0);
    $("#joy-deadman").val("OFF");
    $("#joy-deadman").addClass("danger-danger");
    $("#joy-deadman").removeClass("success-success");
    setTimeout(gpPoller, 500);
  }
}

/**
 * Begin joystick control!
 */
function gpBegin() {
  sendData = false;
  if (typeof gamepad !== "undefined") {
    ajaxSend("beginJoystickControl").success(function (ret) {
      if (ret.indexOf("true") > -1) {
        $("#joy-begin").addClass("hidden");
        $("#joy-end").removeClass("hidden");
        sendData = true;
      }
      console.log(ret);
    });
  }
}