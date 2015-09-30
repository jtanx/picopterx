/**
 * @file gridspace.h
 * @brief Obstacle describes obstacles in space using a voxel system.
 **/

#ifndef _PICOPTERX_GRID_SPACE_H
#define _PICOPTERX_GRID_SPACE_H

#include "waypoints.h"
#include "pathplan.h"
#include <iostream>

namespace picopter {
    class GridSpace{

        public:
            typedef struct voxel {
                int observations;
                bool isFull;
            } Voxel;
            
            typedef struct index3D{
                double x;
                double y;
                double z;
            } index3D;
            
            GridSpace(PathPlan *p /*, FLightController *fc*/);
            void raycast(/*FLightController *fc*/);
            void printToConsole(int rangeMin, int rangeMax, int zDepth);
            void writeImage();
            

            
        private:
            std::vector< std::vector< std::vector<voxel> > > grid;
            double voxelLength;
            double voxelWidth;
            double voxelHeight;
            navigation::Coord3D launchPoint;
            navigation::Coord3D getGPS();
            index3D worldToGrid(navigation::Coord3D GPSloc);
            navigation::Coord3D gridToWorld(index3D loc);
            double degToRad(double deg);
            PathPlan *pathPlan;    
    };
}

#endif // _PICOPTERX_GRID_SPACE_H
