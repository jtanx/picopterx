/**
 * @file env_mapping.h
 * @brief Environmental mapping header.
 **/

#ifndef _PICOPTERX_ENV_MAPPING_H
#define _PICOPTERX_ENV_MAPPING_H

/* For the Options class */
#include "opts.h"
#include "flightcontroller.h"

namespace picopter {
    class EnvironmentalMapping : public FlightTask {
        public:
            EnvironmentalMapping(Options *opts, int radius=7);
            EnvironmentalMapping(int radius=7);
            virtual ~EnvironmentalMapping() override;
            
            void Run(FlightController *fc, void *opts) override;
            bool Finished() override;
        private:
            /** Flag to indicate if we're finished **/
            std::atomic<bool> m_finished;
            /** The rotation radius **/
            int m_radius;


            void GotoLocation(FlightController *fc, navigation::Coord3D l, navigation::Coord3D roi, bool relative_alt);
            /** Copy constructor (disabled) **/
            EnvironmentalMapping(const EnvironmentalMapping &other);
            /** Assignment operator (disabled) **/
            EnvironmentalMapping& operator= (const EnvironmentalMapping &other);
    };
}

#endif // _PICOPTERX_ENV_MAPPING_H
