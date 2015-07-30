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
    std::deque<navigation::Coord2D> m_pts;
    std::shared_ptr<FlightTask> m_user_tracker;
public:
    webInterfaceHandler(Options *opts, std::unique_ptr<picopter::FlightController> &fc)
    : m_opts(opts) 
    , m_fc(fc)
    {
        // Your initialization goes here
    }

    bool beginWaypointsThread()
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            // ALREADY RUNNING
            return false;
        } else {
            std::shared_ptr<FlightTask> wpts = std::make_shared<Waypoints>(m_opts, m_pts, WAYPOINT_NORMAL);
            
            if (!m_fc->RunTask(TASK_WAYPOINTS, wpts, NULL)) {
                return false;
            }
        }
        return true;
    }

    bool beginLawnmowerThread()
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            // ALREADY RUNNING
            return false;
        } else {
            std::shared_ptr<FlightTask> wpts = std::make_shared<Waypoints>(m_opts, m_pts, WAYPOINT_LAWNMOWER);
            
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
        static int counter = 0;
        std::string path = std::string(PICOPTER_HOME_LOCATION "/save_") + std::to_string(counter++) + std::string(".jpg");
        
        if (m_fc->cam) {
            m_fc->cam->TakePhoto(path);
        }
        Log(LOG_DEBUG, "USER MAPPING");
        return false;
    }
    
    bool beginObjectTrackingThread(const int32_t method)
    {
        if (m_fc->GetCurrentTaskId() != TASK_NONE) {
            // ALREADY RUNNING
            return false;
        } else if (m_fc->cam != NULL) {
            int width = m_fc->cam->GetInputWidth(), height = m_fc->cam->GetInputHeight();
            std::shared_ptr<FlightTask> trk = std::make_shared<ObjectTracker>(m_opts, width, height);
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
            m_fc->cam->SetMode(static_cast<CameraStream::CameraMode>(mode));
            return m_fc->cam->GetMode();
        }
        return -1;
    }

    int32_t requestCameraMode() {
        if (m_fc->cam) {
            return m_fc->cam->GetMode();
        }
        return -1;
    }

    bool setCameraLearningSize(bool decrease) {
        if (m_fc->cam) {
            m_fc->cam->SetLearningSize(decrease);
        }
        return true;
    }

    bool showLearningThreshold(bool show) {
        if (m_fc->cam) {
            m_fc->cam->ShowLearningThreshold(show);
            return true;
        }
        return false;
    }

    void doCameraAutoLearning(std::map<std::string, int32_t> & _return) {
        if (m_fc->cam) {
            m_fc->cam->DoAutoLearning(&_return);
        }
    }

    void setCameraLearningValues(std::map<std::string, int32_t> & _return, const std::map<std::string, int32_t> & values)  {
        if (m_fc->cam) {
            m_fc->cam->DoManualLearning(values, &_return);
        }
    }

    int32_t requestLearningHue() {
        if (m_fc->cam) {
            return m_fc->cam->GetLearningHue();
        }
        return 0;
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
        _return = ss.str();
    }

    void requestCoords(coordDeg& _return)
    {
        GPSData d;
        m_fc->gps->GetLatest(&d);
        _return.lat = std::isnan(d.fix.lat) ? -1 : d.fix.lat;
        _return.lon = std::isnan(d.fix.lon) ? -1 : d.fix.lon;
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

    void requestNextWaypoint(coordDeg& _return)
    {
        // Your implementation goes here
        //printf("requestNextWaypoint\n");
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
            navigation::Coord2D wpt;
            Log(LOG_INFO, "%d: (%.6f,%.6f)", i++, v.lat, v.lon);
            wpt.lat = v.lat;
            wpt.lon = v.lon;
            m_pts.push_back(wpt);
        }
        //printf("updateWaypoints\n");
        return true;
    }

    bool resetWaypoints()
    {
        // Your implementation goes here
        //printf("resetWaypoints\n");
        return false;
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

