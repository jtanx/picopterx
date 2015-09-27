/**
 * @file mavcommstcp.cpp
 * @brief Implementation of the MAVCommsTCP class.
 */

#include "common.h"
#include "mavcommslink.h"

#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <sys/time.h>

using namespace picopter;

/**
 * Constructor. Establishes a MAVLink connection via TCP/IP.
 * @param [in] address The IPv4 address to connect to.
 * @param [in] port The port to connect to.
 * @throws std::invalid_argument on error.
 */
MAVCommsTCP::MAVCommsTCP(const char *address, uint16_t port)
: m_address(address)
, m_port(port)
, m_fd(-1)
, m_packet_drop_count(0)
{
    struct sockaddr_in addr = {0};

    m_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_fd == -1) {
        throw std::invalid_argument("Could not create socket.");
    } else if (inet_pton(AF_INET, address, &addr.sin_addr) != 1) {
        throw std::invalid_argument("Could not parse address.");
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (connect(m_fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        throw std::invalid_argument("Could not connect to server.");
    }
}

/**
 * Destructor. Closes the TCP/IP connection.
 */
MAVCommsTCP::~MAVCommsTCP() {
    if (close(m_fd) == -1) {
        Log(LOG_WARNING, "Could not close TCP socket.");
    }
}

/**
 * Reads a MAVLink message.
 * @param [in] ret The location to store the read message, if any.
 * @return true iff a message was read.
 */
bool MAVCommsTCP::ReadMessage(mavlink_message_t *ret) {
    struct timeval timeout = {5,0}; //5 second timeout
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
 * Writes a MAVLink message. 
 * @param [in] src The message to be sent.
 * @return true iff the whole message was sent.
 */
bool MAVCommsTCP::WriteMessage(const mavlink_message_t *src) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(buffer, src);
    size_t ret = write(m_fd, buffer, length);

    if (ret < length) {
        Log(LOG_DEBUG, "Failed to write %d bytes, count: %zu", length, ret);
        return false;
    }

    return true;
}