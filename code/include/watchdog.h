/**
 * @file watchdog.h
 * @brief Simple watchdog class to perform an action on a timeout condition.
 */

#ifndef _PICOPTERX_WATCHDOG_H
#define _PICOPTERX_WATCHDOG_H

#include <functional>

namespace picopter {
    class Watchdog {
        public:
            Watchdog(int timeout, std::function<void()> cb);
            virtual ~Watchdog();
            
            void Start();
            void Stop();
            void Touch();
        private:
            /** Worker thread. **/
            std::thread m_worker;
            /** End indicator. **/
            std::atomic<bool> m_stop;
            /** Watchdog index. **/
            std::atomic<int> m_index;
            /** The watchdog timeout, in milliseconds. **/
            int m_timeout;
            /** Handle to the callback if a timeout occurs. **/
            std::function<void()> m_callback;
            
            /** Worker thread **/
            void Worker();
            /** Copy constructor (disabled) **/
            Watchdog(const Watchdog &other);
            /** Assignment operator (disabled) **/
            Watchdog& operator= (const Watchdog &other);
    };
}

#endif // _PICOPTERX_WATCHDOG_H