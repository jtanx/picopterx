/**
 * @file picopter.cpp
 * @brief The main entry point to the server.
 */

#include "picopter.h"
#include "webInterface.h"
#include <arpa/inet.h>
#include <thrift/concurrency/ThreadManager.h>
#include <thrift/concurrency/PosixThreadFactory.h>
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TThreadPoolServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>
#include <sstream>
#include <csignal>

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;
using namespace ::apache::thrift::concurrency;

using boost::shared_ptr;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

using namespace  ::picopter;

std::unique_ptr<TThreadPoolServer> g_server(nullptr);
std::unique_ptr<picopter::FlightController> g_fc(nullptr);

class webInterfaceHandler : virtual public webInterfaceIf
{
private:
    /** User-specified options **/
    Options *m_opts;
    /** Our flight controller **/
    const std::unique_ptr<picopter::FlightController> &m_fc;
    
    /** Our list of waypoints **/
    std::deque<Waypoints::Waypoint> m_pts;
    /** Our list of exclusion zones **/
    std::deque<std::deque<navigation::Coord3D>> m_zones;
    /** A handle to our user tracker (if any) to update the user position. **/
    std::shared_ptr<FlightTask> m_user_tracker;
    /** A handle to our joystick control (if any) to update joystick inputs. **/
    std::shared_ptr<FlightTask> m_joystick_control;
    /** Thread to take pictures (may be unused now) **/
    std::thread m_camera_thread;
    /** Flag to stop camera picture thread (may be unused now) **/
    /** Tertiary mutex to control access to the GridSpace object. **/
    std::mutex m_gridspace_mutex;
    /** Thread to perform GridSpace ray tracing **/
    std::thread m_gridspace_thread;
    GridSpace m_grid;
    std::atomic<bool> m_camera_stop;
    std::atomic<bool> m_stop;
    /** The image sequence number for picture taking (may be unused now) **/
    int m_camera_sequence;
public:
    webInterfaceHandler(Options *opts, std::unique_ptr<picopter::FlightController> &fc)
    : m_opts(opts)
    , m_fc(fc)
    , m_grid(fc.get())
    , m_camera_stop{false}
    , m_stop{false}
    , m_camera_sequence(0)
    {
        // Your initialization goes here
        m_gridspace_thread = std::thread(&webInterfaceHandler::gridSpaceLoop, this);
    }

    ~webInterfaceHandler() {
        m_camera_stop = true;
        m_stop = true;
        
        if (m_camera_thread.joinable()) {
            m_camera_thread.join();
        }
        
        m_gridspace_thread.join();
    }
    
    bool beginTakeoff(int alt)
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            //ALREADY RUNNING
            return false;
        } else {
            std::shared_ptr<FlightTask> utl = std::make_shared<UtilityModule>(
                m_opts, UtilityModule::UTILITY_TAKEOFF);
            if (!m_fc->RunTask(TASK_UTILITY, utl, reinterpret_cast<void*>(alt))) {
                return false;
            }
            return true;
        }
    }
    
    bool beginReturnToLaunch()
    {
        m_fc->Stop();
        return m_fc->fb->DoReturnToLaunch();
    }

    bool beginWaypointsThread(int mode)
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            // ALREADY RUNNING
            return false;
        } else {
            switch(mode) {
                case WAYPOINT_LAWNMOWER:
                    if (m_pts.size() != 2) {
                        return false;
                    }
                break;
                
                case WAYPOINT_SPIRAL: case WAYPOINT_SPIRAL_OUT:
                    if (m_pts.size() < 2) {
                        return false;
                    } else if (CoordDistance(m_pts[0].pt, m_pts[1].pt) < 0.5) {
                        return false; //Radius too small to do spiral pattern.
                    } else if (m_pts.size() >= 3 && CoordDistance(m_pts[0].pt, m_pts[2].pt) < 0.5) {
                        return false; //Radius too small to do spiral pattern.
                    }
                break;
                
                case WAYPOINT_NORMAL:
                break;
                
                default: //Don't accept unkown modes.
                    return false;
                
            }
            
            std::shared_ptr<FlightTask> wpts = std::make_shared<Waypoints>(
                m_opts, m_pts, m_zones, &m_grid, static_cast<WaypointMethod>(mode));
            if (!m_fc->RunTask(TASK_WAYPOINTS, wpts, NULL)) {
                return false;
            }
        }
        return true;
    }

    bool beginUserTrackingThread()
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            // ALREADY RUNNING
            return false;
        } else {
            m_user_tracker = std::make_shared<UserTracker>(m_opts);
            if (!m_fc->RunTask(TASK_USER_TRACKING, m_user_tracker, NULL)) {
                return false;
            }
        }
        return false;
    }
    
    bool beginJoystickControl()
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            //ALREADY RUNNING
            return false;
        } else {
            m_joystick_control = std::make_shared<UtilityModule>(
                m_opts, UtilityModule::UTILITY_JOYSTICK);
            if (!m_fc->RunTask(TASK_UTILITY, m_joystick_control, NULL)) {
                return false;
            }
            return true;
        }
    }
    
    bool beginPicturesThread()
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            // ALREADY RUNNING
            return false;
        } else if (m_fc->cam == NULL) {
            Log(LOG_WARNING, "Not running pictures without camera!");
            return false;
        } else {
            std::shared_ptr<FlightTask> utl = std::make_shared<UtilityModule>(
                m_opts, UtilityModule::UTILITY_PICTURES);
            if (!m_fc->RunTask(TASK_UTILITY, utl, NULL)) {
                return false;
            }
        }
        return true;
    }

    bool beginUserMappingThread(bool isauto, int radius)
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            // ALREADY RUNNING
            return false;
        } else if (m_fc->cam == NULL || m_fc->lidar == NULL) {
            Log(LOG_WARNING, "Not running mapping without camera and LIDAR!");
            return false; //Cannot run without camera and LIDAR
        } else {
            Log(LOG_DEBUG, "Running environmental mapping!");
            if (isauto) {
                radius = m_fc->lidar->GetLatest() / 100;
            }
            
            if (radius < 3 || radius > 15) {
                Log(LOG_WARNING, "Not mapping using radius of %d!", radius);
                return false;
            }
            
            std::shared_ptr<FlightTask> mapper = 
                std::make_shared<EnvironmentalMapping>(m_opts, radius);
            if (!m_fc->RunTask(TASK_ENVIRONMENTAL_MAPPING, mapper, NULL)) {
                return false;
            }
        }
        return true;

        /*
        if (m_camera_thread.joinable()) {
            m_camera_stop = true;
            m_camera_thread.join();
            m_camera_stop = false;
            Log(LOG_DEBUG, "USER MAPPING STOP");
            return false;grid
        } else {
            m_camera_thread = std::thread([this] {
                Log(LOG_DEBUG, "THREAD START %d", m_camera_sequence);
                for (int counter = 0; !m_camera_stop;) {
                    std::string path = std::string(PICOPTER_HOME_LOCATION "/pics/save_") +
                    std::to_string(m_camera_sequence) + "_" +
                    std::to_string(counter) + std::string(".jpg");

                    if (m_fc->cam) {
                        if (m_fc->cam->TakePhoto(path)) {
                            counter++;
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(35));
                }
                m_camera_sequence++;
            });

            Log(LOG_DEBUG, "USER MAPPING");
        }
        return true;
        */
    }

    bool beginObjectTrackingThread(const int32_t method)
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            // ALREADY RUNNING
            return false;
        } else if (m_fc->cam != NULL) {
            std::shared_ptr<FlightTask> trk = std::make_shared<ObjectTracker>(m_opts);
            if (method == 1) {
                static_cast<ObjectTracker*>(trk.get())->SetTrackMethod(ObjectTracker::TRACK_ROTATE);
            }

            if (!m_fc->RunTask(TASK_OBJECT_TRACKING, trk, NULL)) {
                return false;
            }
        } else {
            Log(LOG_WARNING, "Not running object tracking - no camera!");
        }
        return true;
    }

    int32_t setCameraMode(int32_t mode) {
        if (m_fc->cam) {
            Log(LOG_INFO, "Mode: %d", mode);
            return m_fc->cam->SetMode(static_cast<CameraStream::CameraMode>(mode));
        }
        return -1;
    }

    int32_t requestCameraMode() {
        if (m_fc->cam) {
            return m_fc->cam->GetMode();
        }
        return -1;
    }

    bool doCameraAutoLearning() {
        if (m_fc->cam) {
            m_fc->cam->DoAutoLearning();
            return true;
        }
        return false;
    }

    void requestCameraConfig(std::string& _return) {
        if (m_fc->cam) {
            Options config;
            m_fc->cam->GetConfig(&config);
            _return = config.Serialise();
        } else {
            _return = "{}";
        }
    }

    bool setCameraConfig(const std::string& config) {
        if (m_fc->cam) {
            Options opts;
            opts.Merge(config.c_str());
            m_fc->cam->SetConfig(&opts);
            return true;
        }
        return false;
    }

    bool allStop()
    {
        m_fc->Stop();
        return true;
    }

    void requestStatus(std::string& _return)
    {
        std::stringstream ss;
        ss << (*m_fc);
        if (m_camera_thread.joinable()) {
            ss << " (Capturing photos)";
        }
        _return = ss.str();
    }

    void requestCoords(coordDeg& _return)
    {
        GPSData d;
        m_fc->gps->GetLatest(&d);
        _return.lat = std::isnan(d.fix.lat) ? -1 : d.fix.lat;
        _return.lon = std::isnan(d.fix.lon) ? -1 : d.fix.lon;
        _return.alt = std::isnan(d.fix.alt) || std::isnan(d.fix.groundalt) ?
            -1 : d.fix.alt - d.fix.groundalt;
        //printf("requestCoords %f,%f\n", _return.lat, _return.lon);
    }

    void requestSettings(std::string& _return) {
        if (m_opts) {
            _return = m_opts->Serialise();
        } else {
            _return = "{}";
        }
    }

    bool updateSettings(const std::string& settings) {
        //Only update if we're not running anything...
        if (m_fc->GetCurrentTaskId() == TASK_NONE && m_opts) {
            if (m_opts->Merge(settings.c_str())) {
                return m_fc->ReloadSettings(m_opts);
            }
        }
        return false;
    }

    double requestBearing()
    {
        if (m_fc->imu) {
            double bearing = m_fc->imu->GetLatestYaw();
            if (std::isnan(bearing)) {
                bearing = 0;
            } else if (bearing < 0) {
                bearing += 360;
            }
            return bearing;
        } else {
            GPSData d;
            m_fc->gps->GetLatest(&d);
            if (!std::isnan(d.fix.bearing)) {
                return d.fix.bearing;
            }
            return 0;
        }
    }
    
    double requestLidar()
    {
        if (m_fc->lidar) {
            return m_fc->lidar->GetLatest() / 100.0;
        }
        return -1;
    }

    void requestAttitude(attitude& _return)
    {
        if (m_fc->imu) {
            IMUData d;
            m_fc->imu->GetLatest(&d);
            _return.roll = d.roll;
            _return.pitch = d.pitch;
            _return.yaw = d.yaw;
        } else {
            _return.roll = _return.pitch = _return.yaw = 0;
        }
    }
    
    bool updateUserPosition(const coordDeg& wpt)
    {
        std::shared_ptr<FlightTask> trk(m_user_tracker);
        if (trk) {
            if (trk->Finished()) {
                //Task is finished, remove our reference to it
                m_user_tracker.reset();
            } else {
                navigation::Coord2D uwpt;
                uwpt.lat = wpt.lat;
                uwpt.lon = wpt.lon;

                static_cast<UserTracker*>(trk.get())->UpdateUserPosition(uwpt);
                return true;
            }
        }
        return false;
    }
    
    bool updateJoystick(int throttle, int yaw, int x, int y)
    {
        std::shared_ptr<FlightTask> joy(m_joystick_control);
        if (joy) {
            if (joy->Finished()) {
                //Task is finished, remove our reference to it
                m_joystick_control.reset();
            } else {
                static_cast<UtilityModule*>(joy.get())->UpdateJoystick(
                    throttle, yaw, x, y);
                return true;
            }
        }
        return false;
    }

    bool updateWaypoints(const std::vector<coordDeg> & wpts)
    {
        Log(LOG_INFO, "Updating waypoints");

        int i = 1;
        m_pts.clear();
        for (coordDeg v : wpts) {
            Waypoints::Waypoint wpt{};
            Log(LOG_INFO, "%d: (%.6f,%.6f,%.1f)", i++, v.lat, v.lon, v.alt);
            wpt.pt.lat = v.lat;
            wpt.pt.lon = v.lon;
            wpt.pt.alt = v.alt;
            m_pts.push_back(wpt);
        }
        //printf("updateWaypoints\n");
        return true;
    }
    
    bool updateExclusions(const std::vector< std::vector<coordDeg> > & zones) {
        Log(LOG_INFO, "Updating exclusion zones");
        
        //TODO: Update with Richard's actual code.
        int i = 1;
        m_zones.clear();
        for (std::vector<coordDeg> z : zones) {
            std::deque<navigation::Coord3D> zone;
            int j = 1;
            for (coordDeg v : z) {
                zone.push_back(navigation::Coord3D{v.lat, v.lon, v.alt});
                Log(LOG_INFO, "%d:%d: (%.6f, %.6f, %.1f)",
                    i, j++, v.lat, v.lon, v.alt);
            }
            i++;
            m_zones.push_back(zone);
        }
        return false;
    }
    
private:
    void gridSpaceLoop() {
        while (!m_stop) {
            if (m_fc->gps->HasFix()) {
                m_grid.raycast(m_fc.get());
                m_grid.writeImage();
            }
            sleep_for(milliseconds(1000));
        }
    }
};

/**
 * SIGINT/SIGTERM interrupt handler
 */
void terminate(int signum) {
    static int killcount = 0;

    if (++killcount > 3) {
        Fatal("Okay you asked for it; exiting immediately!");
    } else {
        Log(LOG_WARNING, "Terminate signal received! Attempting termination...");

        if (!g_fc) {
            Fatal("Flight controller was not initialised; exiting immediately!");
        } else if (!g_server) {
            Fatal("Server was not initialised; exiting immediately!");
        }

        g_server->stop();
        sleep_for(milliseconds(400));
        if (g_fc) {
            g_fc->Stop();
        }
    }
}

int main(int argc, char **argv)
{
    Options *opts = NULL;
    LogInit();

    //We need options irregardless.
    if (argc > 1) {
        opts = new Options(argv[1]);
    } else {
        opts = new Options();
    }

    //Signal handlers
    struct sigaction signal_handler;	
    signal_handler.sa_handler = terminate;
    sigemptyset(&signal_handler.sa_mask);
    signal_handler.sa_flags = 0;

    sigaction(SIGTERM, &signal_handler, NULL);
    sigaction(SIGINT,  &signal_handler, NULL);

    int port = 9090;

    try {
        g_fc.reset(new picopter::FlightController(opts));
    } catch (const std::invalid_argument &e) {
        Fatal("Failed to initialise %s which is required, exiting.", e.what());
    }

    shared_ptr<webInterfaceHandler> handler(new webInterfaceHandler(opts, g_fc));
    shared_ptr<TProcessor> processor(new webInterfaceProcessor(handler));
    shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
    shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
    shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

    shared_ptr<ThreadManager> threadManager(ThreadManager::newSimpleThreadManager(1));
    shared_ptr<PosixThreadFactory> threadFactory(new PosixThreadFactory());
    threadManager->threadFactory(threadFactory);
    threadManager->start();

    g_server.reset(new TThreadPoolServer(processor,serverTransport,transportFactory,protocolFactory,threadManager));
    try {
        Log(LOG_INFO, "Server started.");
        g_server->serve();
    } catch (const TTransportException &e) {
        Fatal("Cannot start server: Thrift port 9090 is already in use.");
    }

    delete opts;
    Log(LOG_INFO, "Server stopped.");
    return 0;
}

