/**
 * @file observations.cpp
 * @brief Utility functions for manipulating and tracking uncertainties in observations
 */

/*
    
*/



#include "flightcontroller.h"
#include "observations.h"
Observation::Observation() {




}


//estimate the probability the given observation  is of the same object
Observations::getSameProbability(Observation* observation){    


}

//calculate the centre and covariance widths of this object
Observations::getUncertaintyParams(){                         


}

//add another sighting to this object
Observations::appendObservation(Observation* observation){     
    sightings.push_back(observation);
    for(int i=0; i<_uncertainty_coeffs; i++){
        uncertainty.coeffs[i] += observation.uncertainty.coeffs[i];
    }

}

//remove an observation from this object
Observations::removeObservation(Observation* observation){     


}
