/**
 * @file utility.cpp
 * @brief Performs utility control operations on the copter (e.g. take-offs).
 */

#include "common.h"
#include "utility.h"

using picopter::UtilityModule;

/**
 * Constructor.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @param [in] method The task which this instance should perform.
 */
UtilityModule::UtilityModule(Options *opts, UtilityMethod method)
: m_finished{false}
, m_method(method)
{
    
}

/**
 * Constructor. Shortcut to UtilityModule(nullptr, method).
 * @param [in] method The task whcih this instance should perform.
 */
UtilityModule::UtilityModule(UtilityMethod method)
: UtilityModule(nullptr, method) {}

/**
 * Destructor.
 */
UtilityModule::~UtilityModule() {
    
}

/**
 * Main execution loop.
 * @param fc The flight controller that initiated this call.
 * @param opts Any user-specified options. For UTILITY_TAKEOFF:
 *             opts is interpreted as the takeoff altitude (cast to int).
 */
void UtilityModule::Run(FlightController *fc, void *opts) {
    Log(LOG_INFO, "Utility module initiated; awaiting authorisation...");
    SetCurrentState(fc, STATE_AWAITING_AUTH);
    if (!fc->WaitForAuth()) {
        Log(LOG_INFO, "All stop acknowledged; quitting!");
        m_finished = true;
        return;
    }
    
    switch(m_method) {
        case UTILITY_TAKEOFF:
            SetCurrentState(fc, STATE_UTILITY_AWAITING_ARM);
            Log(LOG_INFO, "Waiting for motors to be armed before take-off...");
            while (!fc->fb->IsArmed() && !fc->CheckForStop()) {
                fc->Sleep(100);
            }
            
            if (fc->fb->IsArmed() && !fc->CheckForStop()) {
                Log(LOG_INFO, "Performing take-off!");
                SetCurrentState(fc, STATE_UTILITY_TAKEOFF);
                int alt = (int)(intptr_t)opts;
                if (fc->fb->DoGuidedTakeoff(alt)) {
                    while (fc->gps->GetLatestRelAlt() < (alt-0.2) && !fc->CheckForStop()) {
                        fc->Sleep(100);
                    }
                    Log(LOG_INFO, "Takeoff complete!");
                } else {
                    Log(LOG_WARNING, "Could not take-off! Are you already flying?");
                }
            }
            break;
    }
    
    m_finished = true;
}

/**
 * Indicates whether or not the task is finished.
 * @return true iff finished.
 */
bool UtilityModule::Finished() {
    return m_finished;
}