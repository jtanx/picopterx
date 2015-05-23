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

using picopter::FlightBoard;
using picopter::FlightData;

/**
 * Constructor; initiates a connection to ServoBlaster.
 * Assumes that ServoBlaster has already been started and initialised.
 * @param opts A pointer to options, if any (NULL for defaults)
 * @throws std::invalid_argument if it can't connect to ServoBlaster.
 */
FlightBoard::FlightBoard(Options *opts)
: m_currentData{}
{
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
