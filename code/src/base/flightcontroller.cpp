/**
 * @file flightcontroller.cpp
 * @brief Base controller. Ties in all the equipment (sensors and actuators).
 * @todo Add in camera things.
 */

#include "common.h"
#include "flightcontroller.h"
#include "gpio.h"

using namespace picopter;
using namespace picopter::navigation;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using steady_clock = std::chrono::steady_clock;
using std::this_thread::sleep_for;
using namespace std::placeholders;

const int FlightController::SLEEP_PERIOD;

/**
 * Helper method to initialise a base module.
 * @tparam Item The class of the base module to be initialised.
 * @param what A description of what the base module is.
 * @param pt A reference to the pointer where the result will be stored.
 * @param opts A pointer to the options instance, or NULL.
 * @param b The buzzer.
 * @param required Indicated if this module is required.
 * @param tries The number of tries to make.
 */
template <typename Item>
void InitialiseItem(const char *what, Item* &pt, Options *opts, Buzzer *b, bool required, int tries = -1) {
    pt = nullptr;
    for (int i = 0; !pt && (tries < 0 || i < tries); i++) {
        try {
            pt = new Item(opts);
        } catch (const std::invalid_argument &e) {
            if (i+1 < tries) {
                Log(LOG_WARNING, "Failed to initialise %s (%s); retrying in 1 second...", what, e.what());
                b->Play(200, 40, 100);
                sleep_for(milliseconds(1000));
            }
        }
    }
    
    if (!pt) {
        if (required) {
            throw std::invalid_argument(what);
        } else {
            Log(LOG_WARNING, "Failed to initialise %s; skipping.", what);
        }
    }
}

/**
 * The flight controller constructor.
 * Initialises all members as necessary.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @throws std::invalid_argument if a required component fails to initialise.
 */
FlightController::FlightController(Options *opts)
: fb(m_fb)
, imu(m_imu)
, gps(m_gps)
, buzzer(m_buzzer)
, cam(m_camera)
, lidar(m_lidar)
, m_stop{false}
, m_quit{false}
, m_state{STATE_STOPPED}
, m_task_id{TASK_NONE}
, m_hud{}
, m_fb_status_counter(0)
{
    //GPSGPSD *gps;
    m_buzzer = new Buzzer();
    
    InitialiseItem("flight board", m_fb, opts, m_buzzer, true, 3);
    InitialiseItem("LIDAR", m_lidar, opts, m_buzzer, false, 1);
    //InitialiseItem("GPS", gps, opts, m_buzzer, true, 3);
    //m_gps = gps;
    m_imu = m_fb->GetIMUInstance();    
    m_gps = m_fb->GetGPSInstance();
    InitialiseItem("Camera", m_camera, opts, m_buzzer, false, 1);
    if (m_camera) {
        m_camera->SetMode(CameraStream::MODE_CONNECTED_COMPONENTS);
    }
    
    //Register the HUD parser.
    m_fb->RegisterHandler(MAVLINK_MSG_ID_VFR_HUD,
        std::bind(&FlightController::HUDParser, this, _1));
    m_fb->RegisterHandler(MAVLINK_MSG_ID_SYSTEM_TIME,
        std::bind(&FlightController::HUDParser, this, _1));
    m_fb->RegisterHandler(MAVLINK_MSG_ID_STATUSTEXT,
        std::bind(&FlightController::HUDParser, this, _1));
    m_fb->RegisterHandler(MAVLINK_MSG_ID_SYS_STATUS,
        std::bind(&FlightController::HUDParser, this, _1));

    Log(LOG_INFO, "Initialised components!"); 
    m_buzzer->PlayWait(200, 200, 100);
}

/**
 * Constructor. Constructs a new flight controller with default settings.
 */
FlightController::FlightController() : FlightController(NULL) {}

/**
 * Destructor. Stops/cleans up the base components (depends on RAII)
 */
FlightController::~FlightController() {
    m_quit.store(true, std::memory_order_relaxed);
    
    if (m_task_thread.valid()) {
        Log(LOG_INFO, "Waiting for task to end...");
        Stop();
        m_task_thread.wait();
    }
    delete m_camera;
    delete m_fb;
    //delete m_imu; //Part of the FlightBoard now
    //delete m_gps; //Part of the FlightBoard now
    delete m_buzzer;
    delete m_lidar;
}

/**
 * HUD processing callback.
 * @return Return_Description
 */
void FlightController::HUDParser(const mavlink_message_t *msg) {
    if (msg->msgid == MAVLINK_MSG_ID_VFR_HUD) {
        mavlink_vfr_hud_t vfr;
        mavlink_msg_vfr_hud_decode(msg, &vfr);

        m_hud.air_speed = vfr.airspeed;
        m_hud.ground_speed = vfr.groundspeed;
        m_hud.heading = vfr.heading;
        m_hud.throttle = vfr.throttle;
        m_hud.alt_msl = vfr.alt;
        m_hud.climb = vfr.climb;
        
        std::lock_guard<std::mutex> lock(m_control_mutex);
        if (m_camera) {
            GPSData d;
            std::stringstream ss;
            ss << (*this);
            m_gps->GetLatest(&d);
            if (m_lidar) {
                m_hud.lidar = m_lidar->GetLatest() / 100.0f;
            }
            m_hud.pos = Coord3D{d.fix.lat, d.fix.lon, d.fix.alt-d.fix.groundalt};
            m_hud.status1 = ss.str();
            if (m_fb_status_counter < 14 && m_fb_status_text.size() > 0) {
                m_hud.status2 = m_fb_status_text;
            } else {
                m_hud.status2.clear();
            }
            m_fb->GetGimbalPose(&m_hud.gimbal);
            m_camera->SetHUDInfo(&m_hud);
        }
    } else if (msg->msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
        mavlink_system_time_t tm;
        mavlink_msg_system_time_decode(msg, &tm);

        m_hud.unix_time_offset = (tm.time_unix_usec / 1000000) - time(NULL); 
        if (m_hud.unix_time_offset < 0) {
            m_hud.unix_time_offset = 0;
        } else if (m_hud.unix_time_offset > 5) { //If the offset is > 5 seconds
            time_t nt = (tm.time_unix_usec / 1000000);
            if (stime(&nt) == 0) {
                Log(LOG_NOTICE, "System time updated using GPS offset.");
                m_hud.unix_time_offset = 0;
            }
        }
    } else if (msg->msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t st;
        mavlink_msg_statustext_decode(msg, &st);
        st.text[49] = 0; //Null-terminate string. Truncates to 49 chars long...
        m_fb_status_text = st.text;
        m_fb_status_counter = 0;
    } else if (msg->msgid == MAVLINK_MSG_ID_SYS_STATUS) {
        mavlink_sys_status_t status;
        mavlink_msg_sys_status_decode(msg, &status);
        m_hud.batt_voltage = status.voltage_battery*1e-3;
        m_hud.batt_current = status.current_battery*1e-2;
        m_hud.batt_remaining = status.battery_remaining;
    }
    m_fb_status_counter++;
}

/**
 * Send an indication that the flight controller should stop the running task.
 */
void FlightController::Stop() {
    Log(LOG_INFO, "All stop received!");
    m_stop.store(true, std::memory_order_relaxed);
}

/**
 * Check whether a stop should occur or not.
 */
bool FlightController::CheckForStop() {
    return m_quit.load(std::memory_order_relaxed) || 
        m_stop.load(std::memory_order_relaxed) || !m_fb->IsAutoMode();
}

/**
 * Wait for the user to give authorisation for autonomous mode.
 * @return true iff authorisation was given. Will return false when the
 *         'all stop' signal is returned.
 */
bool FlightController::WaitForAuth() {
    static const milliseconds wait(SLEEP_PERIOD);
    bool stop;
    
    while (!(stop = m_stop.load(std::memory_order_relaxed)) && !m_fb->IsAutoMode()){
        sleep_for(wait);
    }
    return !stop;
}

/**
 * Allow for the reloading of settings for *non-essential* items.
 * Settings will not be reloaded on-the-fly for things like the GPS, 
 * flight board, etc. Which are mission-critical.  
 * @param opts The options to reload from.  
 * @return true iff all reloaded components successfully reloaded. 
 */
bool FlightController::ReloadSettings(Options *opts) {
    std::lock_guard<std::mutex> lock(m_control_mutex);
    delete m_camera;
    InitialiseItem("Camera", m_camera, opts, m_buzzer, false, 1);
    if (m_camera) {
        m_camera->SetMode(CameraStream::MODE_CONNECTED_COMPONENTS);
    }
    return static_cast<bool>(m_camera);
}

/**
 * Retrieves the current state of the flight controller.
 * @return The current state.
 */
ControllerState FlightController::GetCurrentState() {
    if (m_fb->IsRTL()) {
        return STATE_RTL;
    }
    return m_state.load(std::memory_order_relaxed);
}

/**
 * Sets the current controller state.
 * Only the flight controller itself and the currently run task should
 * have access to this method.
 * @return The previous state.
 */
ControllerState FlightController::SetCurrentState(ControllerState state) {
    return m_state.exchange(state, std::memory_order_relaxed);
}

/**
 * Retrieves the current ID oft the task being run.
 * @return The current task ID.
 */
TaskIdentifier FlightController::GetCurrentTaskId() {
    return m_task_id.load(std::memory_order_relaxed);
}

/**
 * Perform a checked sleep which can be interrupted by the stop signal.
 * @param ms The sleep time, in milliseconds.
 * @return true iff the sleep completed normally (i.e. not interrupted).
 */
bool FlightController::Sleep(int ms) {
    static const milliseconds sleep_default(SLEEP_PERIOD);
    milliseconds remaining(ms);
    auto now = steady_clock::now();
    auto end = now + remaining;
    bool stop;
    
    while (!(stop = CheckForStop()) && now < end) {
        sleep_for(std::min(sleep_default, remaining));
        now = steady_clock::now();
        remaining = duration_cast<milliseconds>(end - now);
    }
    
    return !stop;
}

/**
 * Runs a given task, if no task is currently being run.
 * @param tid The task identifier of the task to be run.
 * @param task The task instance to be run.
 * @param opts The task-specific options to be passed to its handler.
 * @return true iff the task was started.
 */
bool FlightController::RunTask(TaskIdentifier tid, std::shared_ptr<FlightTask> task, void *opts) {
    std::lock_guard<std::mutex> lock(m_task_mutex);
    TaskIdentifier old_tid = m_task_id.load(std::memory_order_relaxed);
    
    if (old_tid != TASK_NONE) {
        Log(LOG_WARNING, "Task id %d is already running; not running task id %d.",
            old_tid, tid);
        return false;
    } else if (m_task_thread.valid()) {
        Log(LOG_WARNING, "Waiting for previous task to exit...");
        if (m_task_thread.wait_for(milliseconds(200)) != std::future_status::ready) {
            Log(LOG_WARNING, "Wait timed out - giving up.");
            return false;
        }
    }
    
    Log(LOG_INFO, "Running new task with id %d.", tid);
    m_task_id.store(tid, std::memory_order_relaxed);
    m_stop.store(false, std::memory_order_relaxed);
    m_task = task;
    m_task_thread = std::async(std::launch::async, [this, opts, tid] {
        m_task->Run(this, opts);
        m_task.reset();
        
        m_fb->Stop();
        Log(LOG_INFO, "Task with id %d ended.", tid);
        m_state.store(STATE_STOPPED, std::memory_order_relaxed);
        m_stop.store(false, std::memory_order_relaxed);
        m_task_id.store(TASK_NONE, std::memory_order_relaxed);
    });
    
    return true;
}

namespace picopter {
    /**
     * Stream operator of the Flight Controller.
     * @return The current state of the flight controller (description).
     */
    std::ostream& operator<<(std::ostream &stream, FlightController &fc) {
        switch(fc.GetCurrentState()) {
            case STATE_STOPPED:
                stream << "All stop. Standing by."; break;
            case STATE_RTL:
                stream << "All stop. RTL mode."; break;
            case STATE_GPS_WAIT_FOR_FIX:
                stream << "Waiting for a GPS fix."; break;
            case STATE_AWAITING_AUTH:
                stream << "Awaiting auto mode."; break;
            case STATE_INFER_BEARING:
                stream << "Inferring the bearing."; break;
            case STATE_WAYPOINTS_MOVING:
                stream << "Moving to the waypoint."; break;
            case STATE_WAYPOINTS_IDLING:
                stream << "Idling at the current waypoint."; break;
            case STATE_WAYPOINTS_FINISHED:
                stream << "Finished the waypoints navigation."; break;
            case STATE_TRACKING_SEARCHING:
                stream << "Searching for an object to track."; break;
            case STATE_TRACKING_LOCKED:
                stream << "Tracking an object."; break;
            case STATE_TRACKING_USER:
                stream << "Tracking user."; break;
            case STATE_ENV_MAPPING:
                stream << "Performing environmental mapping."; break;
            case STATE_UTILITY_AWAITING_ARM:
                stream << "Awaiting motor arming."; break;
            case STATE_UTILITY_TAKEOFF:
                stream << "Performing takeoff."; break;
            case STATE_UTILITY_JOYSTICK:
                stream << "Under joystick control."; break;
            case STATE_UTILITY_PICTURES:
                stream << "Taking pictures"; break;
        }
        return stream;
    }
}
