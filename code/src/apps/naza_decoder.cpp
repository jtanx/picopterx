/**
 * @file naza_decoder.cpp
 * @brief Sample application to decode and print out NAZA gps data
 */
#include "common.h"
#include "NazaDecoderLib.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <wiringPi.h>
#include <wiringSerial.h>

/**
 * Decode the bytestream and print the decoded message when available.
 * @param decoder The NAZA GPS decoder
 * @param buf The current byte in the raw bytestream
 */
void decodeMessage(NazaDecoderLib &decoder, uint8_t buf) {
    static double last_heading = -1;
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
                    "%02d/%02d/%02d %02d:%02d:%02d,%.07f,%.07f,%.03f,%.03f,%.03f,%.03f,%.02f,%.02f,%d,%d",
                    decoder.getDay(), decoder.getMonth(), decoder.getYear(),
                    decoder.getHour(), decoder.getMinute(), decoder.getSecond(),
                    decoder.getLat(), decoder.getLon(), decoder.getGpsAlt(),
                    decoder.getSpeed(), decoder.getCog(), last_heading, decoder.getHdop(),
                    decoder.getVdop(), decoder.getFixType(),
                    decoder.getNumSat());
                fprintf(stderr, "%s\n", buf);
                printf("%s\n", buf);
            }
        break;
      case NAZA_MESSAGE_COMPASS:
        double inclination = atan2(sqrt(decoder.getMagXVal()*decoder.getMagXVal() +
                                       decoder.getMagYVal()*decoder.getMagYVal()), decoder.getMagZVal());
        fprintf(stderr, "Heading: %.3f, Magnetic Inclination: %.3f\n", decoder.getHeadingNc(), inclination);
        last_heading = decoder.getHeadingNc();
        break;
    }   
}

/**
 * Main entry point.
 */
int main(int argc, char *argv[]) {
    NazaDecoderLib decoder;
    
    if (argc < 2) {
#ifndef IS_ON_PI
        printf("Usage: %s naza_dump\n", argv[0]);
        return 1;
#else
        int fd;
        wiringPiSetup();
        
        fd = serialOpen("/dev/ttyAMA0", 115200);
        if (fd == -1) {
            perror("Could not open serial connection");
            return 1;
        }
        
        while (serialDataAvail(fd) != -1) {
            decodeMessage(decoder, serialGetchar(fd));
        }
#endif
    } else {
        FILE *fp = fopen(argv[1], "rb");
        uint8_t buf;
        if (!fp) {
            perror("Could not open file");
            return 1;
        }
        
        while (fread(&buf, 1, 1, fp) > 0) {
            decodeMessage(decoder, buf);
        }
        fclose(fp);
    }
}