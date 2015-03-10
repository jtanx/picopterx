#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "libgpsmm.h"

int main(int argc, char *argv[]) {
    if (argc < 2) {
        return 1;
    }
    
    gpsmm gps("localhost", DEFAULT_GPSD_PORT);
    if (gps.stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        printf("GPSD error!\n");
        return 1;
    }
    
    FILE *fpo = fopen(argv[1], "w");
    while (1) {
        if (gps.waiting(50000)) {
            struct gps_data_t* data;
            if ((data = gps.read()) == NULL) {
                printf("Failed to read GPS data\n");
            } else if (data->set & LATLON_SET) {
                printf("%.10f, %.10f; %f\n", data->fix.latitude, data->fix.longitude, data->fix.time);
                fprintf(fpo, "%.10f, %.10f; %f\n", data->fix.latitude, data->fix.longitude, data->fix.time);
            }
        }
        fflush(fpo);
    }
    
}


