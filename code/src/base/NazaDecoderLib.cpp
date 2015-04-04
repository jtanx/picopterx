/*
  DJI Naza (v1, v1 Lite, V2) data decoder library
  (c) Pawelsky 20141130
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection

  The RC PWM input code taken from https://www.instructables.com/id/RC-Quadrotor-Helicopter/step12/Arduino-Demo-PWM-Input/
*/

#define _USE_MATH_DEFINES
#include <cmath>
#include "NazaDecoderLib.h"

NazaDecoderLib::NazaDecoderLib()
: payload{0}
, seq(0)
, cnt(0)
, msgId(0)
, msgLen(0)
, cs1(0)
, cs2(0)
, magXMin(0)
, magXMax(0)
, magYMin(0)
, magYMax(0)
{
}

int32_t NazaDecoderLib::decodeLong(uint8_t idx, uint8_t mask)
{
    return ((payload[idx+3] ^ mask) << 24) | ((payload[idx+2] ^ mask) << 16) |
           ((payload[idx+1] ^ mask) << 8) | (payload[idx] ^ mask);
}

int16_t NazaDecoderLib::decodeShort(uint8_t idx, uint8_t mask)
{
    return ((payload[idx+1] ^ mask) << 8) | (payload[idx] ^ mask);
}

void NazaDecoderLib::updateCS(int input)
{
    cs1 += input;
    cs2 += cs1;
}

double NazaDecoderLib::getLat() { return lat; }
double NazaDecoderLib::getLon() { return lon; }
double NazaDecoderLib::getGpsAlt() { return gpsAlt; }
double NazaDecoderLib::getSpeed() { return spd; }
NazaDecoderLib::gps_fix_t  NazaDecoderLib::getFixType() { return fix; }
uint8_t NazaDecoderLib::getNumSat() { return sat; }
double NazaDecoderLib::getHeadingNc() { return headingNc; }
double NazaDecoderLib::getCog() { return cog; }
double NazaDecoderLib::getGpsVsi() { return gpsVsi; }
double NazaDecoderLib::getHdop() { return hdop; }
double NazaDecoderLib::getVdop() { return vdop; }
uint8_t NazaDecoderLib::getYear() { return year; }
uint8_t NazaDecoderLib::getMonth() { return month; }
uint8_t NazaDecoderLib::getDay() { return day; }
uint8_t NazaDecoderLib::getHour() { return hour; }
uint8_t NazaDecoderLib::getMinute() { return minute; }
uint8_t NazaDecoderLib::getSecond() { return second; }
int16_t NazaDecoderLib::getMagXval() { return magXVal; }
int16_t NazaDecoderLib::getMagYval() { return magYVal; }
int16_t NazaDecoderLib::getMagZval() { return magZVal; }

uint8_t NazaDecoderLib::decode(int input)
{ 
    if((seq == 0) && (input == 0x55)) { seq++; }                                                            // header (part 1 - 0x55)
    else if((seq == 1) && (input == 0xAA)) { cs1 = 0; cs2 = 0; seq++; }                                     // header (part 2 - 0xAA) 
    else if(seq == 2) { msgId = input; updateCS(input); seq++; }                                            // message id
    else if((seq == 3) && (((msgId == 0x10) && (input == 0x3A)) ||                                          // message payload length (should match message id)
                         ((msgId == 0x20) && (input == 0x06)))) { msgLen = input; cnt = 0; updateCS(input); seq++; }
    else if(seq == 4) { payload[cnt++] = input; updateCS(input); if(cnt >= msgLen) { seq++; } }             // store payload in buffer
    else if((seq == 5) && (input == cs1)) { seq++; }                                                        // verify checksum #1
    else if((seq == 6) && (input == cs2)) { seq++; }                                                        // verify checksum #2
    else seq = 0; 

    if(seq == 7) // all data in buffer
    {
        seq = 0;
        // Decode GPS data
        if(msgId == NAZA_MESSAGE_GPS)
        {
            uint8_t mask = payload[55];
            
            /*
            //http://www.rcgroups.com/forums/showpost.php?p=26210591&postcount=15
            uint8_t mask = (payload[48]^(payload[56]>>4)) & 0x0F;
            mask = (mask^(payload[48]<<3)) & 0x7F;
            mask = mask^(mask<<7);
            */
            
            uint32_t time = decodeLong(0, mask);
            second = time & 0x3F; time >>= 6;
            minute = time & 0x3F; time >>= 6;
            hour = time & 0xF; time >>= 4;
            day = time & 0x1F; time >>= 5; if(hour > 7) day++;
            month = time & 0xF; time >>= 4;
            year = time & 0x7F;
            lon = (double)decodeLong(4, mask) / 10000000;
            lat = (double)decodeLong(8, mask) / 10000000;
            gpsAlt = (double)decodeLong(12, mask) / 1000;
            double nVel = (double)decodeLong(28, mask) / 100; 
            double eVel = (double)decodeLong(32, mask) / 100;
            spd = sqrt(nVel * nVel + eVel * eVel);
            cog = atan2(eVel, nVel) * 180.0 / M_PI;
            if(cog < 0) cog += 360.0;
            gpsVsi = -(double)decodeLong(36, mask) / 100;
            vdop = (double)decodeShort(42, mask) / 100;
            double ndop = (double)decodeShort(44, mask) / 100; 
            double edop = (double)decodeShort(46, mask) / 100;
            hdop = sqrt(ndop * ndop + edop * edop);
            sat  = payload[48];
            uint8_t fixType = payload[50] ^ mask;
            uint8_t fixFlags = payload[52] ^ mask;
            switch(fixType)
            {
                case 2 : fix = FIX_2D; break;
                case 3 : fix = FIX_3D; break;
                default: fix = NO_FIX; break;
            }
            if((fix != NO_FIX) && (fixFlags & 0x02)) fix = FIX_DGPS;
        }
        // Decode compass data (not tilt compensated)
        else if (msgId == NAZA_MESSAGE_COMPASS)
        {
            uint8_t mask = payload[4];
            mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7));

            int16_t x = decodeShort(0, mask);
            int16_t y = decodeShort(2, mask);

            //three axis compass according to http://www.rcgroups.com/forums/showpost.php?p=26248426&postcount=62
            int16_t z = ((payload[5] ^ mask) << 8) | (payload[4]);  //can't use decodeShort() because byte 9 is not XORed

            //this makes sense, but it's pretty crude. The copter will need to do a full rotation before it finds the centres of its magnetometers
            //if we want to be really clever, we should probably store these values somwhere.
            if(x > magXMax) magXMax = x;
            if(x < magXMin) magXMin = x;
            if(y > magYMax) magYMax = y;
            if(y < magYMin) magYMin = y;
            if(y > magYMax) magYMax = z;
            if(y < magYMin) magYMin = z;

            int16_t magXMid = (magXMax + magXMin)/2;
            int16_t magYMid = (magYMax + magYMin)/2;
            int16_t magZMid = (magZMax + magZMin)/2;

            magXVal = x - magXMid;
            magYVal = y - magYMid;
            magZVal = z - magZMid;

            headingNc = -atan2(magYVal, magXVal) * 180.0 / M_PI;

            if(headingNc < 0) headingNc += 360.0; 
        }
        return msgId;
    }

    return NAZA_MESSAGE_NONE;
}
