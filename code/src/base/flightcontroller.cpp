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
 * @throws std::invalid_argument if a required component fails to initialise.
 */
FlightController::FlightController(Options *opts)
: fb(m_fb)
, imu(m_imu)
, gps(m_gps)
, buzzer(m_buzzer)
, cam(m_camera)
, m_stop{false}
, m_state{STATE_STOPPED}
, m_task_id{TASK_NONE}
, m_task{nullptr}
{
    GPSNaza *gps;
    m_buzzer = new Buzzer();
    
    InitialiseItem("flight board", m_fb, opts, m_buzzer, true, 3);
    InitialiseItem("GPS", gps, opts, m_buzzer, true, 3);
    InitialiseItem("IMU", m_imu, opts, m_buzzer, false, 1);
    InitialiseItem("Camera", m_camera, opts, m_buzzer, false, 1);
    
    m_gps = gps;
    if (m_camera) {
        m_camera->Start();
        m_camera->SetMode(CameraStream::MODE_CONNECTED_COMPONENTS);
    }
    
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
    if (m_task_thread.valid()) {
        Log(LOG_INFO, "Waiting for task to end...");
        Stop();
        m_task_thread.wait();
    }
    delete m_task;
    delete m_fb;
    delete m_imu;
    delete m_gps;
    delete m_buzzer;
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
    return m_stop.load(std::memory_order_relaxed) || !gpio::IsAutoMode();
}

/**
 * Wait for the user to give authorisation for autonomous mode.
 * @return true iff authorisation was given. Will return false when the
 *         'all stop' signal is returned.
 */
bool FlightController::WaitForAuth() {
    static const milliseconds wait(SLEEP_PERIOD);
    bool stop;
    
    while (!(stop = m_stop.load(std::memory_order_relaxed)) && !gpio::IsAutoMode()){
        sleep_for(wait);
    }
    return !stop;
}

/**
 * Retrieves the current state of the flight controller.
 * @return The current state.
 */
ControllerState FlightController::GetCurrentState() {
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
 * Infer the current bearing by moving forwards and using the GPS heading.
 * @param ret The return location (bearing in radians).
 * @param move_time The time spent moving forwards in ms (default is 5000ms).
 * @return true iff the bearing could be inferred.
 */
bool FlightController::InferBearing(double *ret, int move_time) {
    GPSData start, end;
    double dist_moved;
    
    Log(LOG_INFO, "Inferring the current bearing...");
    SetCurrentState(STATE_INFER_BEARING);
    
    m_fb->Stop();
    if (!m_gps->WaitForFix(200)) {
        Log(LOG_INFO, "Bearing inferral failed - no GPS fix.");
        SetCurrentState(STATE_STOPPED);
        return false;
    }
    
    m_gps->GetLatest(&start);   
    m_fb->SetElevator(40);
    Sleep(move_time);
    m_fb->Stop();
    m_gps->GetLatest(&end);
    
    if ((dist_moved = navigation::CoordDistance(start.fix, end.fix)) < 1.0) {
        Log(LOG_INFO, "Bearing inferral failed - did not move far enough (%.1f m)", dist_moved);
        SetCurrentState(STATE_STOPPED);
        return false;
    }
    
    Log(LOG_INFO, "The inferred bearing is: %.2f deg (%.2f deg, %.2f m)", 
        end.fix.heading, navigation::CoordBearing(start.fix, end.fix), dist_moved);
    *ret = end.fix.heading;
    SetCurrentState(STATE_STOPPED);
    return true;
}

/**
 * Runs a given task, if no task is currently being run.
 * @param tid The task identifier of the task to be run.
 * @param task The task instance to be run.
 * @param opts The task-specific options to be passed to its handler.
 * @return true iff the task was started.
 */
bool FlightController::RunTask(TaskIdentifier tid, FlightTask *task, void *opts) {
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
    m_task_thread = std::async(std::launch::async, [this, task, opts, tid] {
        task->Run(this, opts);
        delete m_task;
        m_task = nullptr;
        
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
        }
        return stream;
    }
}
