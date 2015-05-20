<?php
	$defaultOptions = array(
		"GLOBAL" => array(
			"DEMO_MODE" => NULL,
			"OBSERVATION_MODE" => NULL
		),
		"CAMERA_STREAM" => array(
			"MIN_HUE" => NULL,
			"MAX_HUE" => NULL,
			"MIN_SAT" => NULL,
			"MAX_SAT" => NULL,
			"MIN_VAL" => NULL,
			"MAX_VAL" => NULL,
			"INPUT_WIDTH" => NULL,
			"INPUT_HEIGHT" => NULL,
			"PROCESS_WIDTH" => NULL,
			"STREAM_WIDTH" => NULL,
			"BOX_SIZE" => NULL,
			"LEARN_SIZE" => NULL,
			"LEARN_HUE_WIDTH" => NULL,
			"THREAD_SLEEP_TIME" => NULL,
			"DILATE_ELEMENT" => NULL,
			"ERODE_ELEMENT" => NULL,
			"PIXEL_THRESHOLD" => NULL
		),
		"GPS" => array(
			"FIX_TIMEOUT" => NULL,
			"CYCLE_TIMEOUT" => NULL
		),
		"OBJECT_TRACKER" => array(
			"TRACK_Kpw" => NULL,
			"TRACK_Kpx" => NULL,
			"TRACK_Kpy" => NULL,
			"TRACK_Kpz" => NULL,
			"TRACK_TauIw" => NULL,
			"TRACK_TauIx" => NULL,
			"TRACK_TauIy" => NULL,
			"TRACK_TauIz" => NULL,
			"TRACK_TauDw" => NULL,
			"TRACK_TauDx" => NULL,
			"TRACK_TauDy" => NULL,
			"TRACK_TauDz" => NULL,
			"TRACK_SPEED_LIMIT_W" => NULL,
			"TRACK_SPEED_LIMIT_X" => NULL,
			"TRACK_SPEED_LIMIT_Y" => NULL,
			"TRACK_SPEED_LIMIT_Z" => NULL,
			"TRACK_SETPOINT_W" => NULL,
			"TRACK_SETPOINT_X" => NULL,
			"TRACK_SETPOINT_Y" => NULL,
			"TRACK_SETPOINT_Z" => NULL
		),
		"WAYPOINTS" => array(
			"SIMPLE_Kpxy" => NULL,
			"SIMPLE_Trxy" => NULL,
			"SIMPLE_Tdxy" => NULL,
			"SIMPLE_UPDATE_INTERVAL" => NULL,
			"SIMPLE_WAYPOINT_RADIUS" => NULL,
			"SIMPLE_WAYPOINT_CONTROL_RANGE" => NULL,
			"SIMPLE_WAYPOINT_IDLE_TIME" => NULL,
			"SIMPLE_SPEED_LIMIT" => NULL
		)
	);
	
	
	/**
	 * Filter to the known set of options.
	 * @param $opts The input options
	 * @return The filtered set of options
	 */
	function filterOptions($opts) {
		global $defaultOptions;
		$ret = array();
		
		foreach ($opts as $fk => $fv) {
			if (isset($defaultOptions[$fk])) {
				$ret[$fk] = array();
				foreach ($fv as $k => $v) {
					print $defaultOptions[$fk][$k];
					if (array_key_exists($k, $defaultOptions[$fk])) {
						$ret[$fk][$k] = $v;
					}
				}
			}
		}
		
		return $ret;
	}
?>