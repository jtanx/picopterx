/**
 * @file gpio.h
 * @brief gpio class defines
 */

#ifndef _GPIO_H
#define _GPIO_H

/* Calibration defines */
#define AILERON_CHANNEL 0
#define AILERON_PIN_PHYSICAL 11
#define AILERON_PWM_MID 152			//Calibrated on October 17th
#define AILERON_PWM_SWING 41
#define AILERON_SPEED_MID 0
#define AILERON_SPEED_SWING 100

#define ELEVATOR_CHANNEL 1
#define ELEVATOR_PIN_PHYSICAL 12
#define ELEVATOR_PWM_MID 153		//Calibrated on October 17th
#define ELEVATOR_PWM_SWING 42
#define ELEVATOR_SPEED_MID 0
#define ELEVATOR_SPEED_SWING 100

#define RUDDER_CHANNEL 2
#define RUDDER_PIN_PHYSICAL 15
#define RUDDER_PWM_MID 152			//Calibrated on October 17th
#define RUDDER_PWM_SWING 42
#define RUDDER_SPEED_MID 0
#define RUDDER_SPEED_SWING 100

#define GIMBAL_CHANNEL 3
#define GIMBAL_PIN_PHYSICAL 16
#define GIMBAL_PWM_LOW 95			//Calibrated on October 5th
#define GIMBAL_PWM_HIGH 210
#define GIMBAL_ANGLE_LOW 0
#define GIMBAL_ANGLE_HIGH 90

namespace picopter {
    /**
     * Controls the GPIO pins on the rPi, including PWM functionality.
     */
    namespace gpio {
        const int MODE_PIN = 5;
        const int BUZZER_PIN = 2;

        void init();
        bool isAutoMode();
        void setBuzzer(bool value);
        void setPWM(int aileron, int elevator, int rudder, int gimbal);
    }
}

#endif //_GPIO_H