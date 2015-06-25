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
#include "navigation.h"
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
, m_system_id(0)
, m_component_id(0)
, m_flightboard_id(128) //Arbitrary value 0-255
, m_is_auto_mode{false}
{
    m_link = new MAVCommsSerial("/dev/ttyAMA0", 115200);
    //m_link = new MAVCommsTCP("127.0.0.1", 5760);
    //m_link = new MAVCommsSerial("/dev/virtualcom0", 57600);
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
    delete m_link;
}

void FlightBoard::InputLoop() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    bool initted = false;

    while (!m_shutdown) {
        if (m_link->ReadMessage(&msg)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    printf("Heartbeat!\n");
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                    
                    m_is_auto_mode = static_cast<bool>(heartbeat.custom_mode == GUIDED);
                    printf("Mode: %d, %d\n", heartbeat.custom_mode, (int)m_is_auto_mode);
                    
                    if (!initted) {
                        mavlink_message_t smsg;

                        initted = true;
                        m_system_id = msg.sysid;
                        m_component_id = msg.compid;
                        Log(LOG_DEBUG, "Got sysid: %d, compid: %d", msg.sysid, msg.compid);
                        
                        mavlink_msg_param_request_list_pack(
                            m_system_id, m_flightboard_id, &msg,
                            msg.sysid, msg.compid);
                        Log(LOG_DEBUG, "Sending parameter list request");
                        m_link->WriteMessage(&smsg);
                        
                        //2 Hz update rate
                        mavlink_msg_request_data_stream_pack(
                            m_system_id, m_flightboard_id, &smsg,
                            msg.sysid, msg.compid, MAV_DATA_STREAM_ALL, 2, 1);
                        Log(LOG_DEBUG, "Sending data request");
                        m_link->WriteMessage(&smsg);
                    }
                } break;
                case MAVLINK_MSG_ID_SYS_STATUS: {
                    mavlink_sys_status_t status;
                    mavlink_msg_sys_status_decode(&msg, &status);
                    printf("BATTERY: %.2fV, Draw: %.2fA, Remain: %3d%%\n",
                        status.voltage_battery*1e-3,
                        status.current_battery*1e-2,
                        status.battery_remaining);
                } break;
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);
                    printf("Lat: %.2f, Lon: %.2f, RelAlt: %.2f, Brng: %.2f, (%.1f,%.1f,%.1f)\n",
                    pos.lat*1e-7, pos.lon*1e-7, pos.relative_alt*1e-3, pos.hdg*1e-2,
                    pos.vx*1e-2,pos.vy*1e-2,pos.vz*1e-2);
                } break;
                /*case MAVLINK_MSG_ID_GPS_RAW_INT: {
                    mavlink_gps_raw_int_t gps;
                    printf("GPS!\n");
                    mavlink_msg_gps_raw_int_decode(&msg, &gps);
                    printf("Lat: %.2f, Lon: %.2f, Alt: %.2fm\n",
                        gps.lat * 1e-7, gps.lon * 1e-7, gps.alt / 1000.0);
                } break;*/
                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                    printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
                        RAD2DEG(att.roll), RAD2DEG(att.pitch), RAD2DEG(att.yaw));
                } break;
                case MAVLINK_MSG_ID_PARAM_VALUE: {
                    mavlink_param_value_t param;
                    mavlink_msg_param_value_decode(&msg, &param);
                    Log(LOG_DEBUG, "%.16s", param.param_id);
                } break;
                default: {
                    printf("MSGID: %d\n", msg.msgid);
                } break;
            }
            fflush(stdout);
        }
    }
}

/**
 * Determines if the system is in auto mode.
 * This means that the Pixhawk is in guided mode. Where we can send commands. 
 * @return true iff in auto mode. 
 */
bool FlightBoard::IsAutoMode() {
    return m_is_auto_mode;
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
