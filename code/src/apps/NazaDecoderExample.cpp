/*
  DJI Naza (v1, v1 Lite, V2) data decoder library example
  (c) Pawelsky 20141109
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection
*/

#include "common.h"
#include "NazaDecoderLib.h"
#include <wiringPi.h>
#include <wiringSerial.h>

#ifndef ATTITUDE_SENSING_DISABLED
uint32_t currTime, attiTime;
#endif

int main(int argc, char *argv[]) {
    int fd;
    NazaDecoder decoder;
    wiringPiSetup();
    
    fd = serialOpen("/dev/ttyAMA0", 115200);
    while (serialDataAvail(fd) >= 0) {
        uint8_t decodedMessage = NazaDecoder.decode(serialGetChar(fd));
        switch (decodedMessage)
        {
          case NAZA_MESSAGE_GPS:
            Serial.print("Lat: "); Serial.print(NazaDecoder.getLat(), 7);
            Serial.print(", Lon: "); Serial.print(NazaDecoder.getLon(), 7);
            Serial.print(", Alt: "); Serial.print(NazaDecoder.getGpsAlt(), 7);
            Serial.print(", Fix: "); Serial.print(NazaDecoder.getFixType());
            Serial.print(", Sat: "); Serial.println(NazaDecoder.getNumSat());
            break;
          case NAZA_MESSAGE_COMPASS:
            Serial.print("Heading: "); Serial.println(NazaDecoder.getHeadingNc(), 2);
            break;
        }
    }
}