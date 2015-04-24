/**
 * @file gps_naza.h
 * @brief GPS header for the NAZA implementation.
 */

#ifndef _PICOPTERX_GPS_NAZA_H
#define _PICOPTERX_GPS_NAZA_H

#include "gps_feed.h"

class NazaDecoderLib;

namespace picopter {
    /* Forward declaration of the options class */
    class Options;
    
    /**
     * Class that interacts with the GPS.
     */
    class GPSNaza : public GPS {
        public:
            GPSNaza();
            GPSNaza(Options *opts);
            virtual ~GPSNaza() override;
        private:
            /** The file descriptor to the serial port **/
            int m_fd;
            /** If we had a fix beforehand **/
            bool m_had_fix;
            /** The worker thread **/
            std::thread m_worker;
            /** The NAZA decoder **/
            NazaDecoderLib *m_decoder;
            /** The data logger **/
            DataLog m_log;
            
            /** Copy constructor (disabled) **/
            GPSNaza(const GPSNaza &other);
            /** Assignment operator (disabled) **/
            GPSNaza& operator= (const GPSNaza &other);
            void GPSLoop();
    };
}

#endif // _PICOPTERX_GPS_NAZA_H