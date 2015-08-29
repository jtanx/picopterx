/**
 * @file observations.h
 * @brief The header file for the observations code.
 */

#ifndef _PICOPTERX_OBSERVATIONS_H
#define _PICOPTERX_OBSERVATIONS_H

#include "opts.h"
#include "flightcontroller.h"
#include "camera_stream.h"

namespace picopter {

    //a 3d gaussian elliptical structure
    const int _distrib_coeffs = 10;
    typedef struct Distrib {
        double coeffs[10];
    } Distrib;

    //parameters to construct the elliptic structure above
    typedef struct DistribParams {
        double x,y,z;
        double sigma_x, sigma_y, sigma_z;
        double yaw, pitch, roll;
    } DistribParams;


    //where the observation came from
    typedef enum Source {   //should probably be a collection of booleans instead of splitting sensor values
        CAMERA_BLOB,
        CAMERA_SIFT,
        LIDAR,
        TELEM
    } Source;
    //ObjectInfo    //detections from the camera stream (grouped into single objects)

    typedef struct Observation {
        Distrib distrib;    //3D probability density function (elliptical normal)
        Source source;              //sensor pack the detection came from
        ObjectInfo* camDetection;   //information from camera detection

    } Observation;

    Distrib generatedistrib(DistribParams params);             //generate an distrib struct from a primitive and operators
    DistribParams getdistribParams(Distrib A);                  //calculate the centre and covariance widths of this object

    Distrib combineDistribs(Distrib A, Distrib B);         //combine two distrib distributions (as though statistically independent)
    Distrib translateDistrib(Distrib A, double x, double y, double z);      //translate an distrib struct from the origin
    Distrib rotateDistrib(Distrib A, double yaw, double pitch, double roll);       //translate an distrib struct about the origin
    Distrib stretchDistrib(Distrib A, double sx, double sy, double sz);         //stretch an distrib struct about the origin


    class Observations {
        public:
            Observations();
            double getSameProbability(Observation* observation);    //estimate the probability the given observation is of the same object
            void appendObservation(Observation* observation);     //add another sighting to this object
            void removeObservation(Observation* observation);     //remove an observation from this object

        private:
            Distrib distrib;    //cumulative distrib
            std::vector<Observation*> sightings;


    };
}



#endif  // _PICOPTERX_OBSERVATIONS_H