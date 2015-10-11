#!/usr/bin/env python3
from __future__ import print_function
import sys,os,re
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint
from datetime import datetime
import time
import pytz

time_fmt = r"%d/%m/%Y %H:%M:%S"
#gps_fmt = re.compile(r"^([^:]+ \d+:\d+:\d+): \(([-\.\d]+), ([-\.\d]+)\) \[([-\.\d]+) at ([-\.\d]+)\] \(([-\.\d]+) m\)$")
gps_fmt = re.compile(r"^([^:]+ \d+:\d+:\d+): \(([^,]+), ([^,]+), ([^)]+)\) \[([^\]]+)\]$");

def parse_time(t):
    ctime = datetime(*(time.strptime(t, time_fmt)[0:6]),
                     tzinfo=pytz.timezone("Australia/Perth"))
    return ctime.astimezone(pytz.utc)
    
def main(gps_log):
    with open(gps_log) as fp:
        gpx = GPX()
        
        # Create track of raw GPS log:
        gpx_track = GPXTrack(name="GPS Log")
        gpx.tracks.append(gpx_track)

        # Create segment in GPX track:
        gpx_segment = GPXTrackSegment()
        gpx_track.segments.append(gpx_segment)

        for l in fp:
            m = re.match(gps_fmt, l)
            if m:
                ctime = parse_time(m.group(1))
                lat = float(m.group(2))
                lon = float(m.group(3))
                #speed = float(m.group(4))
                bearing = m.group(5)
                #alt = float(m.group(6))
                speed = 0
                alt = 0
                
                gpx_segment.points.append(GPXTrackPoint(lat,lon,time=ctime, speed=speed, elevation=alt, name=bearing))
        
        with open("gps_log.gpx", "w") as fpo:
            print(gpx.to_xml(), file=fpo)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: %s gps.log" % sys.argv[0])
    else:
        main(sys.argv[1])
        print("Parse complete! Saved to gps_log.gpx")


