/*
  DJI Naza (v1, v1 Lite, V2) data decoder library example
  (c) Pawelsky 20141109
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection
*/

#include "common.h"
#include "NazaDecoderLib.h"
#include <algorithm>
#include <wiringPi.h>
#include <wiringSerial.h>

/**
 * Decode the bytestream and calculate magnetometer calibration values
 * @param decoder The NAZA GPS decoder
 * @param buf The current byte in the raw bytestream
 */
void magCalibrate(NazaDecoderLib &decoder, uint8_t buf) {
    static int16_t magXMax = 0, magXMin = 0,
                   magYMax = 0, magYMin = 0,
                   magZMax = 0, magZMin = 0;
    double magXMid,magYMid,magZMid;
    double magAvgDia, magXScale, magYScale, magZScale;
    double magX,magY,magZ;
    
    uint8_t decodedMessage = decoder.decode(buf);
    switch (decodedMessage)
    {
      case NAZA_MESSAGE_COMPASS:
        magXMin = std::min(decoder.getMagXRaw(), magXMin);
        magXMax = std::max(decoder.getMagXRaw(), magXMax);
        magYMin = std::min(decoder.getMagYRaw(), magYMin);
        magYMax = std::max(decoder.getMagYRaw(), magYMax);
        magZMin = std::min(decoder.getMagZRaw(), magZMin);
        magZMax = std::max(decoder.getMagZRaw(), magZMax);
        
        magAvgDia = ((magXMax-magXMin) + (magYMax-magYMin) + (magZMax-magZMin)) / 3.0;
        magXScale = magAvgDia / (magXMax-magXMin);
        magYScale = magAvgDia / (magYMax-magYMin);
        magZScale = magAvgDia / (magZMax-magZMin);
        
        magXMid = (magXMin+magXMax)/2.0;
        magYMid = (magYMin+magYMax)/2.0;
        magZMid = (magZMin+magZMax)/2.0;
        
        magX = (decoder.getMagXRaw() - magXMid) * magXScale;
        magY = (decoder.getMagYRaw() - magYMid) * magYScale;
        magZ = (decoder.getMagZRaw() - magZMid) * magZScale;
        
        fprintf(stderr, "%d,%d,%d\n", decoder.getMagXRaw(), decoder.getMagYRaw(), decoder.getMagZRaw());
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               magXMin,magXMax,magYMin,magYMax,magZMin,magZMax,decoder.getMagXRaw(), decoder.getMagYRaw(), decoder.getMagZRaw(),
               magX,magY,magZ,magAvgDia,magXScale,magYScale,magZScale);
        
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
            magCalibrate(decoder, serialGetchar(fd));
        }
#endif
    } else {
        FILE *fp = fopen(argv[1], "rb");
        uint8_t buf;
        if (!fp) {
            perror("Could not open file");
            return 1;
        }
        
        printf("MagXMin,MagXMax,MagYMin,MagYMax,MagZmin,MagZMax,MagXRaw,MagYRaw,MagZRaw,MagX,MagY,MagZ,MagAvgRadius,MagXScale,MagYScale,MagZScale\n");
        while (fread(&buf, 1, 1, fp) > 0) {
            magCalibrate(decoder, buf);
        }
        fclose(fp);
    }
}
