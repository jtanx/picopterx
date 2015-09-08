/**
 * @file utility.h
 * @brief The header file for the utility module.
 */

#ifndef _PICOPTERX_UTILITY_H
#define _PICOPTERX_UTILITY_H

/* For the Options class */
#include "opts.h"
#include "flightcontroller.h"
#include "navigation.h"
 
namespace picopter {
    /**
     * Utility class to perform tasks like take-offs etc.
     */
    class UtilityModule : public FlightTask {
        public:
            typedef enum {
                UTILITY_TAKEOFF
            } UtilityMethod;
            
            UtilityModule(Options *opts, UtilityMethod method);
            UtilityModule(UtilityMethod method);
            virtual ~UtilityModule() override;
            
            void Run(FlightController *fc, void *opts) override;
            bool Finished() override;
        private:
            std::atomic<bool> m_finished;
            UtilityMethod m_method;
            
            /** Copy constructor (disabled) **/
            UtilityModule(const UtilityModule &other);
            /** Assignment operator (disabled) **/
            UtilityModule& operator= (const UtilityModule &other);
    };
}

#endif // _PICOPTERX_UTILITY_H