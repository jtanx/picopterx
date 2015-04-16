#!/usr/bin/env python3

import sys,os,re,math
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint
from datetime import datetime
import time
import pytz

time_fmt = r"%d/%m/%y %H:%M:%S"
    
def main(naza_log):
    with open(naza_log) as fp:
        gpx = GPX()
                    
        # Create track of raw GPS log:
        gpx_track = GPXTrack(name="GPS Log")
        gpx.tracks.append(gpx_track)

        # Create segment in GPX track:
        gpx_segment = GPXTrackSegment()
        gpx_track.segments.append(gpx_segment)

        for l in fp:
            row = l.split(",")
            ctime = datetime.strptime(row[0], time_fmt)
            lat, lon, alt = float(row[1]), float(row[2]), float(row[3])
            speed, cog, calheading = float(row[4]), float(row[5]), float(row[6])
            hdop, vdop = float(row[7]), float(row[8])
            fixtype, numsat = int(row[9]), int(row[10])
                
            gpx_segment.points.append(GPXTrackPoint(lat,lon,alt,
                horizontal_dilution=hdop, vertical_dilution=vdop,
                speed=speed, name=str(calheading), time=ctime))
        
        with open("naza_log.gpx", "w") as fpo:
            print(gpx.to_xml(), file=fpo)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: nazadecoder dump.bin > dump.txt\nUsage: %s dump.txt" % sys.argv[0])
    else:
        main(sys.argv[1])
        print("Parse complete! Saved to naza_log.gpx")


