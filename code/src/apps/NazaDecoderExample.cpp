/*
  DJI Naza (v1, v1 Lite, V2) data decoder library example
  (c) Pawelsky 20141109
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection
*/

#include "common.h"
#include "NazaDecoderLib.h"
#include <cmath>
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
    double last_heading = -1;
    
    while (fread(&buf, 1, 1, fp) > 0) {
        uint8_t decodedMessage = decoder.decode(buf);
        switch (decodedMessage)
        {
            case NAZA_MESSAGE_GPS:
                if (decoder.getFixType() == NazaDecoderLib::NO_FIX) {
                    fprintf(stderr, "%02d/%02d/%02d %02d:%02d:%02d: No fix\n" ,
                       decoder.getDay(), decoder.getMonth(), decoder.getYear(),
                       decoder.getHour(), decoder.getMinute(), decoder.getSecond());
                } else {
                    char buf[BUFSIZ];
                    
                    sprintf(buf,
                        "%02d/%02d/%02d %02d:%02d:%02d,%.07f,%.07f,%.07f,%.07f,%.07f,%.02f,%.02f,%d,%d",
                        decoder.getDay(), decoder.getMonth(), decoder.getYear(),
                        decoder.getHour(), decoder.getMinute(), decoder.getSecond(),
                        decoder.getLat(), decoder.getLon(), decoder.getGpsAlt(),
                        decoder.getSpeed(), last_heading, decoder.getHdop(),
                        decoder.getVdop(), decoder.getFixType(),
                        decoder.getNumSat());
                    fprintf(stderr, "%s\n", buf);
                    printf("%s\n", buf);
                }
            break;
          case NAZA_MESSAGE_COMPASS:
            float inclination = atan2(sqrt( (float)decoder.getMagXval()*(float)decoder.getMagXval()
                                          + (float)decoder.getMagYval()*(float)decoder.getMagYval() ), decoder.getMagZval());
            fprintf(stderr, "Heading: %.3f, Magnetic Inclination: %.3f\n", decoder.getHeadingNc(), inclination);
            last_heading = decoder.getHeadingNc();
            break;
        }
    }
    fclose(fp);
}