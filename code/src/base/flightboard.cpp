/**
 * @file flightboard.cpp
 * @brief Controls the output to the flight board.
 * Calculates the correct PWM pulse width to be sent to the PWM drivers
 * for the aileron, elevator and rudder, as well as camera gimbal.
 * Uses wiringPi and a GPIO pin for driving the buzzer with software based PWM.
 */

#include "picopter.h"
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
, m_activated(false)
{
    m_fp = fopen("/dev/servoblaster", "wb");
    if (m_fp == NULL) {
        throw std::invalid_argument("Could not connect to servoblaster");
    }
    actuate(true);
}

/** 
 * Constructor. Constructs a new flight board with default settings.
 */
FlightBoard::FlightBoard() : FlightBoard(NULL) {}

/**
 * Destructor. Closes the connection to ServoBlaster.
 */
FlightBoard::~FlightBoard() {
    fclose(m_fp);
}

/**
 * Initialises actuation of the hexacopter
 */
void FlightBoard::start() {
    m_activated = true;
}

/**
 * Stops the hexacopter and deactivates control.
 * Note: Stopping refers to making it hold its current position.
 * Sets all speeds and the gimbal angle to 0.
 */
void FlightBoard::stop() {
    memset(&m_currentData, 0, sizeof(FlightData));
    actuate();
    
    m_activated = false;
}

/**
 * Returns a copy of the current flight data.
 * @param d A pointer to the output location.
 */
void FlightBoard::getData(FlightData *d) {
    *d = m_currentData;
}

/**
 * Sets the flight data and actuates the hexacopter.
 * @param d Specifies the flight data for how the hexacopter should be actuated.
 */
void FlightBoard::setData(FlightData *d) {
    m_currentData.aileron = picopter::clamp(d->aileron, -100, 100);
    m_currentData.elevator = picopter::clamp(d->elevator, -100, 100);
    m_currentData.rudder = picopter::clamp(d->rudder, -100, 100);
    m_currentData.gimbal = picopter::clamp(d->gimbal, 0, 90);
    
    actuate();
}

/**
 * Sets the aileron speed.
 * @param speed The aileron speed, as a percentage (-100% to 100%)
 */
void FlightBoard::setAileron(int speed) {
    m_currentData.aileron = picopter::clamp(speed, -100, 100);
    setChannelChecked(AILERON_CHANNEL, AILERON_SCALE(m_currentData.aileron));
}

/**
 * Sets the elevator speed.
 * @param speed The elevator speed, as a percentage (-100% to 100%)
 */
void FlightBoard::setElevator(int speed) {
    //Elevator speed is inverted.
    m_currentData.elevator = picopter::clamp(-speed, -100, 100);
    setChannelChecked(ELEVATOR_CHANNEL, ELEVATOR_SCALE(m_currentData.elevator));
}

/**
 * Sets the rudder speed.
 * @param speed The rudder speed, as a percentage (-100% to 100%)
 */
void FlightBoard::setRudder(int speed) {
    m_currentData.rudder = picopter::clamp(speed, -100, 100);
    setChannelChecked(RUDDER_CHANNEL, RUDDER_SCALE(m_currentData.rudder));
}

/**
 * Sets the gimbal angle.
 * @param pos The gimbal angle, in degrees (0 to 90)
 */
void FlightBoard::setGimbal(int pos) {
    m_currentData.gimbal = picopter::clamp(pos, 0, 90);
    setChannelChecked(GIMBAL_CHANNEL, GIMBAL_SCALE(m_currentData.gimbal));
}

/**
 * Tells ServoBlaster to set the pulse width for a given channel.
 * Will only perform the actuation if the we're currently activated.
 * @param channel The ServoBlaster channel to alter
 * @param value The pulse width, in steps
 */
void FlightBoard::setChannelChecked(int channel, int value) {
    if (m_activated) {
        fprintf(m_fp, "%d=%d", channel, value);
    }
}

/**
 * Tells ServoBlaster to set the pulse width for a given channel.
 * @param channel The ServoBlaster channel to alter
 * @param value The pulse width, in steps
 */
void FlightBoard::setChannel(int channel, int value) {
    fprintf(m_fp, "%d=%d", channel, value);
}

/**
 * Actuates all channels using current flight data.
 */
void FlightBoard::actuate(bool force) {
    if (m_activated || force) {
        setChannel(AILERON_CHANNEL, AILERON_SCALE(m_currentData.aileron));
        setChannel(ELEVATOR_CHANNEL, ELEVATOR_SCALE(-m_currentData.elevator));
        setChannel(RUDDER_CHANNEL, RUDDER_SCALE(m_currentData.rudder));
        setChannel(GIMBAL_CHANNEL, GIMBAL_SCALE(m_currentData.gimbal));    
    }
}