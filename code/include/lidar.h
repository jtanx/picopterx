/**
 * @file lidar.h
 * @brief LIDAR range detector header.
 **/

#ifndef _PICOPTERX_LIDAR_H
#define _PICOPTERX_LIDAR_H

/* For the Options class */
#include "opts.h"
#include "datalog.h"

namespace picopter {
    class Lidar {
        public:
            Lidar();
            Lidar(Options *opts);
            virtual ~Lidar(void);
            int GetLatest();
        private:
            int m_fd;
            std::atomic<int> m_distance;
            DataLog m_log;
            
            std::atomic<bool> m_stop;
            std::thread m_worker;
            
            void Worker();
            
            /** Copy constructor (disabled) **/
            Lidar(const Lidar &other);
            /** Assignment operator (disabled) **/
            Lidar& operator= (const Lidar &other);
    };
}

#endif // _PICOPTERX_LIDAR_H