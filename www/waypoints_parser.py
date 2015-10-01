#!/usr/bin/env python3

import sys,os,re
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint
from datetime import datetime
import time
import pytz

time_fmt = r"%d/%m/%Y %H:%M:%S"
gen_fmt = re.compile(r"^([^:]+:[^:]+:[^:]+): ([^:]+): (.*)$")
gps_fmt = re.compile(r"^\(([^,]+), ([^,]+), ([^)]+)\) \[([^\]]+)\]$")
wpt_fmt = re.compile(r"^\(([^,]+), ([^,]+), ([^)]+)\) \[([^,]*),? ?([^,]*),? ?([^\]]*)\]$")

def parse_time(t):
    ctime = datetime(*(time.strptime(t, time_fmt)[0:6]),
                     tzinfo=pytz.timezone("Australia/Perth"))
    return ctime.astimezone(pytz.utc)
    
def main(wpt_log):
    with open(wpt_log) as fp:
        gpx = GPX()
        name, ext = os.path.splitext(wpt_log)
        
        # Create track for the waypoints
        gpx_track = GPXTrack(name="Waypoints log")
        gpx.tracks.append(gpx_track)
        
        # Create segment in GPX track:
        gpx_segment = GPXTrackSegment()
        gpx_track.segments.append(gpx_segment)
        
        #An empty detected object
        det = {}
        
        for l in fp:
            m = re.match(gen_fmt, l)
            if m:
                if m.group(2).startswith("Waypoint"):
                    m2 = re.match(wpt_fmt,m.group(3))
                    if m2:
                        lat = float(m2.group(1))
                        lon = float(m2.group(2))
                        alt = float(m2.group(3))
                        
                        if m2.group(4):
                            roi_lat = float(m2.group(4))
                            roi_lon = float(m2.group(5))
                            roi_alt = float(m2.group(6))
                            #TODO ???
                        wpt = GPXWaypoint(lat, lon, alt,
                                description="W" + m.group(2)[9:],
                                type="Waypoint")
                        gpx.waypoints.append(wpt)
                elif m.group(2) == "At":
                    m2 = re.match(gps_fmt, m.group(3))
                    if m2:
                        ctime = parse_time(m.group(1))
                        lat = float(m2.group(1))
                        lon = float(m2.group(2))
                        alt = float(m2.group(3))
                        try:
                            hdg = m2.group(4)
                        except ValueError:
                            hdg = None
                        gpx_segment.points.append(GPXTrackPoint(lat,lon,alt,time=ctime,comment=hdg))
                elif m.group(2).startswith("Detected"):
                    id = m.group(3)[3:]
                    det = {"ID" : id}
                elif m.group(2) == "Location":
                    m2 = re.match(gps_fmt, m.group(3))
                    if m2:
                        det["time"] = parse_time(m.group(1))
                        det["lat"] = float(m2.group(1))
                        det["lon"] = float(m2.group(2))
                        det["alt"] = float(m2.group(3))
                        try:
                            det["hdg"] = m2.group(4)
                        except ValueError:
                            det["hdg"] = None
                elif m.group(2) == "Image":
                    n = os.path.basename(m.group(3))
                    det["img"] = "pics/" + n
                    
                    poi = GPXWaypoint(det["lat"], det["lon"], det["alt"],
                                time=det["time"], comment=det["hdg"],
                                symbol=det["img"],
                                description="POI" + det["ID"],
                                type="Point of interest")
                    gpx.waypoints.append(poi)
        print(gpx.to_xml())

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: %s waypoints.log" % sys.argv[0])
    else:
        main(sys.argv[1])


