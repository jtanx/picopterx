/**
 * @file navigation.h
 * @brief General navigation code.
 */

#ifndef _PICOPTERX_NAVIGATION_H
#define _PICOPTERX_NAVIGATION_H


namespace picopter {
    /**
     * Holds 2-dimensional geographical coordinate position information.
     */
    typedef struct Coord2D {
        /** Latitude, in degrees. **/
        double lat;
        /** Longitude, in degrees. **/
        double lon;
    } Coord2D;
    
    /**
     * Holds 3-dimensional geographical coordinate position information.
     */
    typedef struct Coord3D {
        /** Latitude, in degrees. **/
        double lat;
        /** Longitude, in degrees. **/
        double lon;
        /** Altitude, in metres. **/
        double alt;
    } Coord3D;
    
    /**
     * Holds a 2-dimensional position in Cartesian space.
     */
    typedef struct Point2D {
        /** x-coordinate. **/
        double x;
        /** y-coordinate **/
        double y;
    } Point2D;
    
    /**
     * Holds a 3-dimensional position in Cartesian space.
     */
    typedef struct Point3D {
        /** x-coordinate. **/
        double x;
        /** y-coordinate **/
        double y;
        /** z-coordinate **/
        double z;
    } Point3D;
    
    /**
     * Holds a set of Euler angles.
     */
    typedef struct EulerAngle {
        /** Roll of the IMU, -pi to pi **/
        double roll;
        /** Pitch of the IMU, -pi to pi **/
        double pitch;
        /** Yaw of the IMU, -pi to pi **/
        double yaw;
    } EulerAngle;
}

#endif // _PICOPTERX_NAVIGATION_H