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
}

service webInterface {
	bool		beginWaypointsThread();
	bool		beginLawnmowerThread();
	bool		beginUserTrackingThread();
	
	bool		allStop();
	
	string		requestStatus();
	coordDeg	requestCoords();
	double		requestBearing();
	
	coordDeg	requestNextWaypoint();
	bool		updateUserPosition(1: coordDeg wpt);
	bool		updateWaypoints(1: list<coordDeg> wpts);
	bool		resetWaypoints();
}
