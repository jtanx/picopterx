/**
 * @file MAVCommsLink.h
 * @brief Defines the interfaces to communicate with a MAVLink device.
 */

#ifndef _PICOPTERX_MAVCOMMSLINK_H
#define _PICOPTERX_MAVCOMMSLINK_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
extern "C" {
    #include <common/mavlink.h>
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
     * @todo Consider removing mutexes (already locked in FlightBoard) 
     */
    class MAVCommsSerial : public MAVCommsLink {
        public:
            MAVCommsSerial(const char *device, int baudrate);
            virtual ~MAVCommsSerial() override;
            bool ReadMessage(mavlink_message_t *ret) override;
            bool WriteMessage(const mavlink_message_t *src) override;
        private:
            std::mutex m_mutex;
            std::string m_device;
            int m_baudrate, m_fd;
            mavlink_status_t m_last_status;

            /** Copy constructor (disabled) **/
            MAVCommsSerial(const MAVCommsSerial &other);
            /** Assignment operator (disabled) **/
            MAVCommsSerial& operator= (const MAVCommsSerial &other);
    };

    /**
     * Establishes a MAVLink communication via TCP/IP. 
     */
    class MAVCommsTCP : public MAVCommsLink {
        public:
            MAVCommsTCP(const char *address);
            virtual ~MAVCommsTCP() override;
            bool ReadMessage(mavlink_message_t *ret) override;
            bool WriteMessage(const mavlink_message_t *src) override;
        private:
            /** Copy constructor (disabled) **/
            MAVCommsTCP(const MAVCommsTCP &other);
            /** Assignment operator (disabled) **/
            MAVCommsTCP& operator= (const MAVCommsTCP &other);
    };
}

#endif // _PICOPTERX_MAVCOMMSLINK_H