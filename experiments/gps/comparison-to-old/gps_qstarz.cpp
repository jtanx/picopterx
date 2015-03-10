#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

typedef struct {
	double time;
	double longitude;
	double latitude;
	int fixQuality;
	int numSatelites;
	double horizDilution;
} GPS_Data;

double nmea2degrees(double nmea) {
	int degrees = (int)(nmea)/100;
	double minutes = nmea - degrees*100;
	return (degrees + minutes/60);
}

int processGPSString(GPS_Data *data, std::string *gpsStrPtr) {
	double nmea_latitude, nmea_longitude;
    char *pt;

	//First check the header
	int i = 0;
	int j = gpsStrPtr->find(",");
	if(gpsStrPtr->compare(i, j-i, "$GPGGA") != 0) {
		return -1;
	}
	
	
	//Time WHY IS THIS INT AND NOT DOUBLE
	i = j+1;
	j = gpsStrPtr->find(",", i);
	int newTime = strtol(gpsStrPtr->substr(i).c_str(), &pt, 10);
    if (*pt && *pt != ',')
        printf("OH SNAP");
    
	//Latitude
	i = j+1;
	j = gpsStrPtr->find(",", i);
    nmea_latitude = strtod(gpsStrPtr->substr(i).c_str(), &pt);
	if (*pt && *pt != ',')
        return -1;
    
	//NS
	i = j+1;
	j = gpsStrPtr->find(",", i);
	if(gpsStrPtr->at(i) != 'N' && gpsStrPtr->at(i) != 'S') {
		return -1;
	}
	if(gpsStrPtr->at(i) == 'N') {
        data->latitude = nmea2degrees(nmea_latitude);
    } else {
        data->latitude = -nmea2degrees(nmea_latitude);
    }
	
	//Longitude
	i = j+1;
	j = gpsStrPtr->find(",", i);
    nmea_longitude = strtod(gpsStrPtr->substr(i).c_str(), &pt);
	if (*pt && *pt != ',')
        return -1;
    
	//EW
	i = j+1;
	j = gpsStrPtr->find(",", i);
	if(gpsStrPtr->at(i) != 'E' && gpsStrPtr->at(i) != 'W') {
		return -1;
	}
    if(gpsStrPtr->at(i) == 'E') {
        data->longitude = nmea2degrees(nmea_longitude);
    } else {
        data->longitude = -nmea2degrees(nmea_longitude);
    }
	
	//Fix quality
	i = j+1;
	j = gpsStrPtr->find(",", i);
    data->fixQuality = strtol(gpsStrPtr->substr(i).c_str(), &pt, 10);
    if (*pt && *pt != ',')
        return -1;	
	
	//Number of satelites
	i = j+1;
	j = gpsStrPtr->find(",", i);
	data->numSatelites = strtol(gpsStrPtr->substr(i).c_str(), &pt, 10);
    if (*pt && *pt != ',')
        return -1;	
	
	//Horizontal dilution
	i = j+1;
	j = gpsStrPtr->find(",", i);
    data->horizDilution = strtod(gpsStrPtr->substr(i).c_str(), &pt);
    if (*pt && *pt != ',')
        return -1;
	
	//Finally update time.
	data->time = newTime;
	return 0;
}


int main(int argc, char *argv[]) {
    for (int i = 1; i < argc; i++) {
        FILE *fp = fopen(argv[i], "r");
        std::string out = (std::string(argv[i]) + ".parsed.txt");
        FILE *fpo = fopen(out.c_str(), "w");
        if (fp && fpo) {
            char buf[BUFSIZ];
            GPS_Data d = {0};
            
            while (fgets(buf, BUFSIZ, fp)) {
                std::string l(buf);
                
                if (!processGPSString(&d, &l)) {
                    fprintf(fpo, "%.10f, %.10f; %f\n", d.latitude, d.longitude, d.time);
                }
            }
            
            fclose(fpo);
            fclose(fp);
        }
    }
}


