/**
 * @file mavcommsserial.cpp
 * @brief Implementation of the MAVCommsSerial class.
 * Based on serial_port.cpp as provided in c_uart_interface_example.
 * https://github.com/mavlink/c_uart_interface_example
 */

#include "common.h"
#include "mavcommslink.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#ifndef B460800
  #define B460800 460800
#endif

#ifndef B921600
  #define B921600 921600
#endif

#ifndef OLCUC
  #define OLCUC 0
#endif

#ifndef ONOEOT
  #define ONOEOT 0
#endif

using namespace picopter;

/**
 * Constructor. Opens a serial connection for MAVLink communication.
 * @param [in] device The serial device, e.g. /dev/ttyUSB0
 * @param [in] baudrate The baud rate, e.g. 57600
 * @throws std::invalid_argument on error. 
 */
MAVCommsSerial::MAVCommsSerial(const char *device, int baudrate)
: m_device(device)
, m_baudrate(baudrate)
, m_fd(-1)
, m_packet_drop_count(0)
{
    struct termios config;

    m_fd = open(m_device.c_str(), O_RDWR|O_NOCTTY);
    if (m_fd == -1) {
        throw std::invalid_argument("Cannot open specified port.");
    } else if (!isatty(m_fd)) {
        throw std::invalid_argument("Not a serial port.");
    }

    //fcntl(m_fd, F_SETFL, 0);
    if (tcgetattr(m_fd, &config) == -1) {
        throw std::invalid_argument("Could not get configuration information.");
    }

    //  Input flags - Turn off input processing
    //  convert break to null byte, no CR to NL translation,
    //  no NL to CR translation, don't mark parity errors or breaks
    //  no input parity check, don't strip high bit off,
    //  no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK|BRKINT|ICRNL|INLCR|PARMRK|INPCK|ISTRIP|IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~(OCRNL|ONLCR|ONLRET|ONOCR|OFILL|OPOST|OLCUC|ONOEOT);

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN|ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE|PARENB);
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 10;

    switch (baudrate) {
        case 1200: baudrate = B1200; break;
        case 1800: baudrate = B1800; break;
        case 9600: baudrate = B9600; break;
        case 19200: baudrate = B19200; break;
        case 38400: baudrate = B38400; break;
        case 57600: baudrate = B57600; break;
        case 115200: baudrate = B115200; break;
        case 460800: baudrate = B460800; break;
        case 921600: baudrate = B921600; break;
        default:
            throw std::invalid_argument("Invalid baudrate.");
    }

    if (cfsetispeed(&config, baudrate) == 1 || cfsetospeed(&config, baudrate) == -1) {
        throw std::invalid_argument("Could not set port baudrate.");
    } else if (tcflush(m_fd, TCIFLUSH) == -1 || tcsetattr(m_fd, TCSANOW, &config) == -1) {
        throw std::invalid_argument("Could not set port configuration.");
    }
}

/**
 * Destructor. Closes the serial link.
 */
MAVCommsSerial::~MAVCommsSerial() {
    if (close(m_fd) == -1) {
        Log(LOG_WARNING, "Could not close serial port: %s", strerror(errno));
    }
}

/**
 * Potentially reads a message from the serial stream.
 * @param [in] ret The location to store the read message, if any.
 * @return true iff a message was read.
 */
bool MAVCommsSerial::ReadMessage(mavlink_message_t *ret) {
    std::lock_guard<std::mutex> lock(m_io_mutex);
    struct timeval timeout = {3,0}; //3 second timeout
    mavlink_status_t status;
    bool received;
    uint8_t cp;
    fd_set read_set;

    FD_ZERO(&read_set);
    FD_SET(m_fd, &read_set);

    if (select(m_fd+1, &read_set, NULL, NULL, &timeout) <= 0) {
        Log(LOG_WARNING, "Select error ocurred.");
        return false;
    } else if (read(m_fd, &cp, 1) < 1) {
        Log(LOG_DEBUG, "Could not read from stream: %s", strerror(errno));
        return false;
    }

    received = mavlink_parse_char(MAVLINK_COMM_0, cp, ret, &status);
    if (status.msg_received == MAVLINK_FRAMING_BAD_CRC) {
        m_packet_drop_count++;
        Log(LOG_DEBUG, "Dropped packets (CRC fail), count: %d", m_packet_drop_count);
    }
    return received;
}

/**
 * Writes a message to the stream.
 * @param [in] src the message to write.
 * @return true iff the whole message was written.
 */
bool MAVCommsSerial::WriteMessage(const mavlink_message_t *src) {
    std::lock_guard<std::mutex> lock(m_io_mutex);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(buffer, src);
    size_t ret = write(m_fd, buffer, length);

    tcdrain(m_fd);
    if (ret < length) {
        Log(LOG_DEBUG, "Failed to write %d bytes, count: %zu", length, ret);
        return false;
    }

    return true;
}