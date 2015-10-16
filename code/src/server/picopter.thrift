/**
 * The first thing to know about are types. The available types in Thrift are:
 *
 *  bool        Boolean, one byte
 *  byte        Signed byte
 *  i16         Signed 16-bit integer
 *  i32         Signed 32-bit integer
 *  i64         Signed 64-bit integer
 *  double      64-bit floating point value
 *  string      String
 *  binary      Blob (byte array)
 *  map<t1,t2>  Map from one type to another
 *  list<t1>    Ordered list of one type
 *  set<t1>     Set of unique elements of one type
 *
 */

namespace cpp picopter
namespace php picopter

struct coordDeg {
	1: double lat,
	2: double lon,
	3: double alt,
}

struct attitude {
	1: double roll,
	2: double pitch,
	3: double yaw,
}

service webInterface {
	bool		beginTakeoff(1: i32 alt);
	bool		beginReturnToLaunch();
	bool		beginWaypointsThread(1: i32 mode);
	bool		beginUserTrackingThread();
	bool		beginObjectTrackingThread(1: i32 method);
	bool		beginPicturesThread();
	bool		beginUserMappingThread(1: bool isauto, 2: i32 radius);
	bool		beginJoystickControl();


	i32			setCameraMode(1: i32 mode);
	i32			requestCameraMode();
	bool		doCameraAutoLearning();
	string		requestCameraConfig();
	bool		setCameraConfig(1:string config);
	
	bool		allStop();
	
	string		requestStatus();
	coordDeg	requestCoords();
	double		requestBearing();
	double		requestLidar();
	attitude	requestAttitude();
	string		requestSettings();
	bool		updateSettings(1: string settings);
	
	bool		updateJoystick(1: i32 throttle, 2: i32 yaw, 3: i32 x, 4: i32 y);
	bool		updateUserPosition(1: coordDeg wpt);
	bool		updateWaypoints(1: list<coordDeg> wpts);
	bool		updateExclusions(1: list<list<coordDeg>> zones);
}
