#!/usr/bin/env python3

import sys,os,re
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint
from datetime import datetime
import time
import pytz

time_fmt = r"%d/%m/%y %H:%M%S"
gps_fmt = re.compile(r"^([\d/]+ \d\d:\d+): ([^,\s]+),([^,\s]+),([^,\s]+),([^,\s]+),([^,\s]+),([^,\s]+?)$")
lawnmower_wpt_fmt = re.compile(r"^([\d/]+ \d\d:\d+): Point (\d+) is ([^\s]+) ([^\s]+)$")
lawnmower_gps_fmt = re.compile(r"^([\d/]+ \d\d:\d+): Currently at ([^\s]+) ([^\s]+)$")

def parse_time(t):
    ctime = datetime(*(time.strptime(t, time_fmt)[0:6]),
                     tzinfo=pytz.timezone("Australia/Perth"))
    return ctime.astimezone(pytz.utc)
    
def main(gps_log, lawnmower_log):
    with open(gps_log) as fp:
        gpx = GPX()
        
        #Fill in the Waypoints and what the lawnmower saw
        with open(lawnmower_log) as fpi:
            # Create track for lawnmower
            gpx_track = GPXTrack(name="Lawmower GPS Log")
            gpx.tracks.append(gpx_track)
            
            # Create segment in GPX track:
            gpx_segment = GPXTrackSegment()
            gpx_track.segments.append(gpx_segment)
            for l in fpi:
                m = re.match(lawnmower_wpt_fmt, l)
                if m:
                    point_num = int(m.group(2))
                    lat = float(m.group(3))
                    lon = float(m.group(4))
                    
                    gpx.waypoints.append(GPXWaypoint(lat,lon,description="P%d" % point_num, type="Lawnmower"))
                
                m = re.match(lawnmower_gps_fmt, l)
                if m:
                    ctime = parse_time(m.group(1))
                    lat = float(m.group(2))
                    lon = float(m.group(3))
                    gpx_segment.points.append(GPXTrackPoint(lat,lon,time=ctime))
                    
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
                #'time' since last DGPS fix is group 4
                hdop = float(m.group(5))
                fix_quality = int(m.group(6))
                num_satellites = int(m.group(7))
                
                gpx_segment.points.append(GPXTrackPoint(lat,lon,horizontal_dilution=hdop,time=ctime))
        
        with open("lawnmower_log.gpx", "w") as fpo:
            print(gpx.to_xml(), file=fpo)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: %s gps.log Lawn_xxxxx.log" % sys.argv[0])
    else:
        main(sys.argv[1], sys.argv[2])
        print("Parse complete! Saved to lawnmower_log.gpx")


