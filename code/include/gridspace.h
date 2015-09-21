/**
 * @file gridspace.h
 * @brief Obstacle describes obstacles in space using a voxel system.
 **/

#ifndef _PICOPTERX_GRID_SPACE_H
#define _PICOPTERX_GRID_SPACE_H

#include "waypoints.h"

namespace picopter {
    class GridSpace{

        public:
            typedef struct voxel {
                std:: vector<bool> observations;
                bool isFull;
            } Voxel;
            
            typedef struct index3D{
                double x;
                double y;
                double z;
            } index3D;
            
            GridSpace();
            
        private:
            std::vector< std::vector< std::vector<voxel> > > grid;
            double copterRadius;
            double copterHeight;
            navigation::Coord3D launchPoint;
            navigation::Coord3D getGPS();
            index3D findCopter();
            double degToRad(double deg);    
    };
}

#endif // _PICOPTERX_GRID_SPACE_H
