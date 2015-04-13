/**
 * @file picopter.cpp
 * @brief The main entry point to the server.
 */

#include "picopter.h"
#include "webInterface.h"
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using boost::shared_ptr;

using namespace  ::picopter;

class webInterfaceHandler : virtual public webInterfaceIf
{
public:
    webInterfaceHandler()
    {
        // Your initialization goes here
    }

    bool beginWaypointsThread()
    {
        // Your implementation goes here
        printf("beginWaypointsThread\n");
        return false;
    }

    bool beginLawnmowerThread()
    {
        // Your implementation goes here
        printf("beginLawnmowerThread\n");
        return false;
    }

    bool beginUserTrackingThread()
    {
        // Your implementation goes here
        printf("beginUserTrackingThread\n");
        return false;
    }

    bool allStop()
    {
        // Your implementation goes here
        printf("allStop\n");
        return false;
    }

    void requestStatus(std::string& _return)
    {
        // Your implementation goes here
        printf("requestStatus\n");
    }

    void requestCoords(coordDeg& _return)
    {
        // Your implementation goes here
        printf("requestCoords\n");
    }

    double requestBearing()
    {
        // Your implementation goes here
        printf("requestBearing\n");
        return false;
    }

    void requestNextWaypoint(coordDeg& _return)
    {
        // Your implementation goes here
        printf("requestNextWaypoint\n");
    }

    bool updateUserPosition(const coordDeg& wpt)
    {
        // Your implementation goes here
        printf("updateUserPosition\n");
        return false;
    }

    bool updateWaypoints(const std::vector<coordDeg> & wpts)
    {
        // Your implementation goes here
        printf("updateWaypoints\n");
        return false;
    }

    bool resetWaypoints()
    {
        // Your implementation goes here
        printf("resetWaypoints\n");
        return false;
    }

};

int main(int argc, char **argv)
{
    int port = 9090;

    shared_ptr<webInterfaceHandler> handler(new webInterfaceHandler());
    shared_ptr<TProcessor> processor(new webInterfaceProcessor(handler));
    shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
    shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
    shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

    LogInit();
    TSimpleServer server(processor, serverTransport, transportFactory, protocolFactory);
    try {
        server.serve();
    } catch (const TTransportException &e) {
        Fatal("Cannot start server: Thrift port 9090 is already in use.");
    }
    return 0;
}

