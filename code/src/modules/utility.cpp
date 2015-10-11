/**
 * @file utility.cpp
 * @brief Performs utility control operations on the copter (e.g. take-offs).
 */

#include "common.h"
#include "utility.h"

using picopter::UtilityModule;
using std::chrono::seconds;

/**
 * Constructor.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @param [in] method The task which this instance should perform.
 */
UtilityModule::UtilityModule(Options *opts, UtilityMethod method)
: m_finished{false}
, m_method(method)
, m_data_available(false)
, m_joystick_data{}
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
    bool wait_longer = false;
    
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
            if (!fc->fb->IsArmed()) {
                wait_longer = true;
            }
            
            while (!fc->fb->IsArmed() && !fc->CheckForStop()) {
                fc->Sleep(100);
            }
            
            //Wait a while for motor spinup, otherwise it will go into land mode.
            if (wait_longer) {
                fc->Sleep(700);
            }
            
            if (fc->fb->IsArmed() && !fc->CheckForStop()) {
                Log(LOG_INFO, "Performing take-off!");
                SetCurrentState(fc, STATE_UTILITY_TAKEOFF);
                int alt = (int)(intptr_t)opts;
                if (fc->fb->DoGuidedTakeoff(alt)) {
                    while (fc->gps->GetLatestRelAlt() < (alt-0.2) && !fc->CheckForStop() && fc->fb->IsArmed()) {
                        fc->Sleep(100);
                    }
                    Log(LOG_INFO, "Takeoff complete!");
                } else {
                    Log(LOG_WARNING, "Could not take-off! Are you already flying?");
                }
            }
            break;
        case UTILITY_JOYSTICK: {
            SetCurrentState(fc, STATE_UTILITY_JOYSTICK);
            Log(LOG_INFO, "Initiating Joystick control!");
            
            std::unique_lock<std::mutex> lock(m_worker_mutex);
            while (!fc->CheckForStop()) {
                m_signaller.wait_for(lock, seconds(1),
                    [this,fc]{return m_data_available || fc->CheckForStop();});
                if (m_data_available) {
                    if (m_joystick_data.w != 0) {
                        fc->fb->SetYaw(m_joystick_data.w, true);
                    }
                    
                    //Don't crash into the ground!!!
                    if (fc->gps->GetLatestRelAlt() < 3 && m_joystick_data.z < 0) {
                        m_joystick_data.z = 0;
                    }
                    fc->fb->SetBodyVel(m_joystick_data);
                    m_data_available = false;
                }
            }
        } break;
        case UTILITY_PICTURES: {
            SetCurrentState(fc, STATE_UTILITY_PICTURES);
            Log(LOG_INFO, "Taking pictures!");
            std::string f = GenerateFilename(
                PICOPTER_HOME_LOCATION "/pics", "utility_pics", "");
        
            int i = 0;
            while (!fc->CheckForStop()) {
                char buf[20];
                sprintf(buf, "-%03d.jpg", i);
        
                if (fc->cam) {
                    if (fc->cam->TakePhoto(f + buf)) {
                        i++;
                    }
                }
                fc->Sleep(200);
            }
        } break;
            
    }
    
    fc->fb->Stop();
    m_finished = true;
}

/**
 * Update the joystick info and inform the worker.
 * @param [in] throttle Throttle percentage (-100% to 100%).
 * @param [in] yaw Yaw percentage (-100% to 100%).
 * @param [in] x Left/Right percentage (-100% to 100%).
 * @param [in] y Fowards/Backwards percentage (-100% to 100%).
 * @return Return_Description
 */
void UtilityModule::UpdateJoystick(int throttle, int yaw, int x, int y) {
    std::unique_lock<std::mutex> lock(m_worker_mutex);
    m_joystick_data.x = picopter::clamp(3.0*x/100.0, -3.0, 3.0);
    m_joystick_data.y = picopter::clamp(3.0*y/100.0, -3.0, 3.0);
    m_joystick_data.z = picopter::clamp(2.0*throttle/100.0, -2.0, 2.0);
    m_joystick_data.w = picopter::clamp(30*yaw/100.0, -30.0, 30.0);
    m_data_available = true;
    lock.unlock();
    m_signaller.notify_one();
}

/**
 * Indicates whether or not the task is finished.
 * @return true iff finished.
 */
bool UtilityModule::Finished() {
    return m_finished;
}
