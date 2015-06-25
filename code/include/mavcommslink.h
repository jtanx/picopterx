/**
 * @file MAVCommsLink.h
 * @brief Defines the interfaces to communicate with a MAVLink device.
 */

#ifndef _PICOPTERX_MAVCOMMSLINK_H
#define _PICOPTERX_MAVCOMMSLINK_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
extern "C" {
    #include <pixhawk/mavlink.h>
    // Auto Pilot Modes enumeration
    enum autopilot_modes {
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        OF_LOITER =    10,  // deprecated
        DRIFT =        11,  // semi-automous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17   // full-brake using inertial/GPS system, no pilot input
    };
}
#pragma GCC diagnostic pop

namespace picopter {
    /**
     * Class to communicate with a MAVLink device. 
     */
    class MAVCommsLink {
        public:
            virtual ~MAVCommsLink() {};
            virtual bool ReadMessage(mavlink_message_t *ret) = 0;
            virtual bool WriteMessage(const mavlink_message_t *src) = 0;
        protected:
            MAVCommsLink() {};
        private:
            /** Copy constructor (disabled) **/
            MAVCommsLink(const MAVCommsLink &other);
            /** Assignment operator (disabled) **/
            MAVCommsLink& operator= (const MAVCommsLink &other);
    };

    /**
     * Establishes a MAVLink communication via serial connection.
     * This class is not thread-safe; the user must ensure this.
     */
    class MAVCommsSerial : public MAVCommsLink {
        public:
            MAVCommsSerial(const char *device, int baudrate);
            virtual ~MAVCommsSerial() override;
            bool ReadMessage(mavlink_message_t *ret) override;
            bool WriteMessage(const mavlink_message_t *src) override;
        private:
            std::string m_device;
            int m_baudrate, m_fd;
            int m_packet_drop_count;

            /** Copy constructor (disabled) **/
            MAVCommsSerial(const MAVCommsSerial &other);
            /** Assignment operator (disabled) **/
            MAVCommsSerial& operator= (const MAVCommsSerial &other);
    };

    /**
     * Establishes a MAVLink communication via TCP/IP.
     * The class is not thread-safe; the user must ensure this.
     */
    class MAVCommsTCP : public MAVCommsLink {
        public:
            MAVCommsTCP(const char *address, uint16_t port);
            virtual ~MAVCommsTCP() override;
            bool ReadMessage(mavlink_message_t *ret) override;
            bool WriteMessage(const mavlink_message_t *src) override;
        private:
            std::string m_address;
            uint16_t m_port;
            int m_fd;
            int m_packet_drop_count;
            
            /** Copy constructor (disabled) **/
            MAVCommsTCP(const MAVCommsTCP &other);
            /** Assignment operator (disabled) **/
            MAVCommsTCP& operator= (const MAVCommsTCP &other);
    };
}

#endif // _PICOPTERX_MAVCOMMSLINK_H