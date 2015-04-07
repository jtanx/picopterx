#!/usr/bin/env python3

import sys,os,re
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint
from datetime import datetime
import time
import pytz

time_fmt = r"%d/%m/%y %H:%M%S"
gps_fmt = re.compile(r"^([\d/]+ \d\d:\d+): ([^,\s]+),([^,\s]+),([^,\s]+),([^,\s]+),([^,\s]+),([^,\s]+?)$")
waypts_gps_fmt = re.compile(r"^([\d/]+ \d\d:\d+): \[WAYPTS\] Currently at ([^\s]+) ([^\s,]+).*$")
waypts_wpt_fmt = re.compile(r"^([\d/]+ \d\d:\d+): \[WAYPTS\].* It has latitude ([^\s]+) and longitude (\d+\.\d+).*$")

def parse_time(t):
    ctime = datetime(*(time.strptime(t, time_fmt)[0:6]),
                     tzinfo=pytz.timezone("Australia/Perth"))
    return ctime.astimezone(pytz.utc)
    
def f7(seq):
    # http://stackoverflow.com/questions/480214/how-do-you-remove-duplicates-from-a-list-in-python-whilst-preserving-order
    seen = set()
    seen_add = seen.add
    return [ x for x in seq if not (x in seen or seen_add(x))]
    
def main(gps_log, waypoints_log):
    with open(gps_log) as fp:
        gpx = GPX()
        
        #Fill in the Waypoints and what the lawnmower saw
        with open(waypoints_log) as fpi:
            # Create track for lawnmower
            gpx_track = GPXTrack(name="Waypoints GPS Log")
            gpx.tracks.append(gpx_track)
            
            # Create segment in GPX track:
            gpx_segment = GPXTrackSegment()
            gpx_track.segments.append(gpx_segment)
            
            waypoints = []
            for l in fpi:
                m = re.match(waypts_wpt_fmt, l)
                if m:
                    waypoints.append((m.group(2), m.group(3)))
                
                m = re.match(waypts_gps_fmt, l)
                if m:
                    ctime = parse_time(m.group(1))
                    lat = float(m.group(2))
                    lon = float(m.group(3))
                    gpx_segment.points.append(GPXTrackPoint(lat,lon,time=ctime))
            
            waypoints = f7(waypoints)
            for i in range(len(waypoints)):
                lat, lon = float(waypoints[i][0]), float(waypoints[i][1])
                gpx.waypoints.append(GPXWaypoint(lat,lon,description="P%d" % (i+1), type="Waypoints"))
                    
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
        
        with open("waypoints_log.gpx", "w") as fpo:
            print(gpx.to_xml(), file=fpo)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: %s gps.log picopter.log" % sys.argv[0])
    else:
        main(sys.argv[1], sys.argv[2])
        print("Parse complete! Saved to waypoints_log.gpx")


