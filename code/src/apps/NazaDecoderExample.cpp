/*
  DJI Naza (v1, v1 Lite, V2) data decoder library example
  (c) Pawelsky 20141109
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection
*/

#include "common.h"
#include "NazaDecoderLib.h"
/*
#include <wiringPi.h>
#include <wiringSerial.h>
*/

int main(int argc, char *argv[]) {
    //int fd;
    NazaDecoderLib decoder;
    //wiringPiSetup();
    
    if (argc < 2) {
        printf("WRONG!\n");
    }
    
    //fd = serialOpen("/dev/ttyAMA0", 115200);
    FILE *fp = fopen(argv[1], "rb");
    uint8_t buf;
    while (fread(&buf, 1, 1, fp) > 0) {
        uint8_t decodedMessage = decoder.decode(buf);
        switch (decodedMessage)
        {
          case NAZA_MESSAGE_GPS:
            printf("Lat: %.3f, Lon: %.3f, Alt: %.3f, Fix: %d, Sat: %d\n",
                   decoder.getLat(), decoder.getLon(), decoder.getGpsAlt(),
                   decoder.getFixType(), decoder.getNumSat());
            printf("time: %d/%d/%d %d:%d:%d\n" ,
                   decoder.getDay(), decoder.getMonth(), decoder.getYear(),
                   decoder.getHour(), decoder.getMinute(), decoder.getSecond());
            break;
          case NAZA_MESSAGE_COMPASS:
            printf("Heading: %.3f\n", decoder.getHeadingNc());
            break;
        }
    }
    fclose(fp);
}