/**
 * @file observations.h
 * @brief The header file for the observations code.
 */

#ifndef _PICOPTERX_OBSERVATIONS_H
#define _PICOPTERX_OBSERVATIONS_H

#include "flightcontroller.h"
#include "camera_stream.h"

namespace picopter {

    //a 3d gaussian elliptical structure
    const int _uncertainty_coeffs = 10;
    typedef struct Uncertainty {
        double coeffs[10];
    } Uncertainty;

    //parameters to construct the elliptic structure above
    typedef struct UncertaintyParams {
        double x,y,z;
        double sigma_x, sigma_y, sigma_z;
        double yaw, pitch, roll;
    } UncertaintyParams;


    //where the observation came from
    typedef enum Source {   //should probably be a collection of booleans instead of splitting sensor values
        CAMERA_BLOB,
        CAMERA_SIFT,
        LIDAR,
        TELEM
    } Source;
    //ObjectInfo    //detections from the camera stream (grouped into single objects)

    typedef struct Observation
    {
        Uncertainty uncertainty;    //3D probability density function (elliptical normal)
        Source source;              //sensor pack the detection came from
        ObjectInfo* camDetection;   //information from camera detection


    };

    Uncertainty generateUncertainty(UncertaintyParams, params);             //generate an uncertainty struct from a primitive and operators
    Uncertainty translateUncertainty(Uncertainty A, Point3D position);      //translate an uncertainty struct from the origin
    Uncertainty rotateUncertainty(Uncertainty A, EulerAngles angles);       //translate an uncertainty struct about the origin
    Uncertainty stretchUncertainty(Uncertainty A, Point3D factors);         //stretch an uncertainty struct about the origin
    UncertaintyParams getUncertaintyParams(Uncertainty A);                  //calculate the centre and covariance widths of this object
    Uncertainty combineUncertainties(Uncertainty A, Uncertainty B);         //combine two uncertainty distributions (as though statistically independent)


    class Observations {
        public:
            
            double getSameProbability(Observation observation);    //estimate the probability the given observation is of the same object
            appendObservation(Observation observation);     //add another sighting to this object
            removeObservation(Observation observation);     //remove an observation from this object

        private:
            Uncertainty uncertainty;    //cumulative uncertainty
            std::vector<Observation> sightings;


    };
}



#endif  // _PICOPTERX_WAYPOINTS_H