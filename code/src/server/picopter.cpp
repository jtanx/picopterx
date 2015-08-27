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
    Options *m_opts;
    const std::unique_ptr<picopter::FlightController> &m_fc;
    std::deque<Waypoints::Waypoint> m_pts;
    std::shared_ptr<FlightTask> m_user_tracker;
    std::thread m_camera_thread;
    std::atomic<bool> m_camera_stop;
    int m_camera_sequence;
public:
    webInterfaceHandler(Options *opts, std::unique_ptr<picopter::FlightController> &fc)
    : m_opts(opts)
    , m_fc(fc)
    , m_camera_stop{false}
    , m_camera_sequence(0)
    {
        // Your initialization goes here
    }

    ~webInterfaceHandler() {
        m_camera_stop = true;
        if (m_camera_thread.joinable()) {
            m_camera_thread.join();
        }
    }

    bool beginWaypointsThread(int mode)
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            // ALREADY RUNNING
            return false;
        } else {
            switch(mode) {
                case WAYPOINT_LAWNMOWER: case WAYPOINT_SPIRAL:
                    if (m_pts.size() != 2) {
                        return false;
                    }
                break;
                
                case WAYPOINT_NORMAL:
                break;
                
                default: //Don't accept unkown modes.
                    return false;
                
            }
            
            std::shared_ptr<FlightTask> wpts = std::make_shared<Waypoints>(m_opts, m_pts, static_cast<WaypointMethod>(mode));
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
        // Your implementation goes here
        //printf("beginUserTrackingThread\n");
        return false;
    }

    bool beginUserMappingThread()
    {
        if (m_camera_thread.joinable()) {
            m_camera_stop = true;
            m_camera_thread.join();
            m_camera_stop = false;
            Log(LOG_DEBUG, "USER MAPPING STOP");
            return false;
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

