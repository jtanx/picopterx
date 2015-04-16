/*
  DJI Naza (v1, v1 Lite, V2) data decoder library example
  (c) Pawelsky 20141109
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection
*/

#include "common.h"
#include "NazaDecoderLib.h"
#include <cmath>
#include <wiringPi.h>
#include <wiringSerial.h>


int main(int argc, char *argv[]) {
    int fd;
    NazaDecoderLib decoder;
    wiringPiSetup();
    
    fd = serialOpen("/dev/ttyAMA0", 115200);
    uint8_t buf;
    
    while (serialDataAvail(fd) >= 0) {
    	buf = serialGetchar(fd);
        uint8_t decodedMessage = decoder.decode(buf);
        switch (decodedMessage) {
          case NAZA_MESSAGE_COMPASS:
          	printf("%.03f\r", decoder.getHeadingNc());
          	//printf("%d,%d,%d\n", decoder.getMagXRaw(), decoder.getMagYRaw(), decoder.getMagZRaw());
          	fflush(stdout);
            break;
        }
    }
    serialClose(fd);
}
