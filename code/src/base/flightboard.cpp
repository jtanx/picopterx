/**
 * @file flightboard.cpp
 * @brief Controls the output to the flight board.
 * Calculates the correct PWM pulse width to be sent to the PWM drivers
 * for the aileron, elevator and rudder, as well as camera gimbal.
 * Uses wiringPi and a GPIO pin for driving the buzzer with software based PWM.
 */

#include "common.h"
#include "flightboard.h"
#include "flightboard-private.h"
#include "mavcommslink.h"

using picopter::FlightBoard;
using picopter::FlightData;

/**
 * Constructor; initiates a connection to the flight computer.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @throws std::invalid_argument if it can't connect to the flight computer.
 */
FlightBoard::FlightBoard(Options *opts)
: m_currentData{}
, m_shutdown{false}
{
    //m_link = new MAVCommsTCP("127.0.0.1", 5760);
    m_link = new MAVCommsSerial("/dev/virtualcom0", 57600);
    m_input_thread = std::thread(&FlightBoard::InputLoop, this);
	Stop();
}

/** 
 * Constructor. Constructs a new flight board with default settings.
 */
FlightBoard::FlightBoard() : FlightBoard(NULL) {}

/**
 * Destructor. Closes the connection to ServoBlaster.
 */
FlightBoard::~FlightBoard() {
    Stop();
    m_shutdown = true;
    m_input_thread.join();
}

void FlightBoard::InputLoop() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    while (!m_shutdown) {
        if (m_link->ReadMessage(&msg)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    printf("Heartbeat!\n");
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                } break;
                case MAVLINK_MSG_ID_GPS_RAW_INT: {
                    mavlink_gps_raw_int_t gps;
                    printf("GPS!\n");
                    mavlink_msg_gps_raw_int_decode(&msg, &gps);
                    printf("Lat: %.2f, Lon: %.2f, Alt: %.2fm\n",
                        gps.lat * 1e-7, gps.lon * 1e-7, gps.alt / 1000.0);
                } break;
                default: {
                    printf("MSGID: %d\n", msg.msgid);
                } break;
            }
        }
    }
}

/**
 * Stops the hexacopter.
 * Note: Stopping refers to making it hold its current position.
 * Sets all speeds and the gimbal angle to 0.
 */
void FlightBoard::Stop() {
    FlightData fd_zero = {0};
    m_currentData = fd_zero;
    Actuate();
}

/**
 * Returns a copy of the current flight data.
 * @param d A pointer to the output location.
 */
void FlightBoard::GetData(FlightData *d) {
    *d = m_currentData;
}

/**
 * Sets the flight data and actuates the hexacopter.
 * @param d Specifies the flight data for how the hexacopter should be actuated.
 */
void FlightBoard::SetData(FlightData *d) {
    m_currentData.aileron = picopter::clamp(d->aileron, -100, 100);
    m_currentData.elevator = picopter::clamp(d->elevator, -100, 100);
    m_currentData.rudder = picopter::clamp(d->rudder, -100, 100);
    m_currentData.gimbal = picopter::clamp(d->gimbal, 0, 90);
    
    Actuate();
}

/**
 * Tells ServoBlaster to set the pulse width for a given channel.
 * Will not be performed until the data is flushed (call FlushData).
 * @param channel The ServoBlaster channel to alter
 * @param value The pulse width, in steps
 */
void FlightBoard::SetChannel(int channel, int value) {

}

/**
 * Sets the aileron speed.
 * @param speed The aileron speed, as a percentage (-100% to 100%)
 */
void FlightBoard::SetAileron(int speed) {
}

/**
 * Sets the elevator speed.
 * @param speed The elevator speed, as a percentage (-100% to 100%)
 */
void FlightBoard::SetElevator(int speed) {

}

/**
 * Sets the rudder speed.
 * @param speed The rudder speed, as a percentage (-100% to 100%)
 */
void FlightBoard::SetRudder(int speed) {
}

/**
 * Sets the gimbal angle.
 * @param pos The gimbal angle, in degrees (0 to 90)
 */
void FlightBoard::SetGimbal(int pos) {
}

/**
 * Actuates all channels using current flight data.
 */
void FlightBoard::Actuate() {
}
