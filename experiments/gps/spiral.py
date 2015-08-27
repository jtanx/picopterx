#!/usr/bin/env python3
import sys,os,re
import distance
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint
from math import cos,sin,pi,ceil

class Coord:
    def __init__(self, lat, lon, alt=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
    def __str__(self):
        return "<%.6f, %.6f, %.1f>" % (self.lat, self.lon, self.alt)

def spiral(c1, c2, c3=None):
    '''Good for small distances only.'''
    if c3 is None:
        c3=c2 #Degenerates to a circle pattern
    
    start_radius = distance.haversine(c1.lat, c1.lon, c2.lat, c2.lon)
    end_radius = distance.haversine(c1.lat, c1.lon, c3.lat, c3.lon)
    start_angle = distance.bearing(c1.lat, c1.lon, c2.lat, c2.lon)
    end_angle = distance.bearing(c1.lat, c1.lon, c3.lat, c3.lon)
    earth_radius = 6378137
    pts = []
    
    climb_rate=min(1, c3.alt-c2.alt) #In m/revolution
    revs = int(ceil((c3.alt-c2.alt)/climb_rate)) if climb_rate != 0 else 1
    radius_delta=(end_radius-start_radius)/revs #Radius increment/revolution
    print("REVS:", revs)
    
    for i in range(revs):        
        j = 0
        while j < 360:
            pct = (i + j/360.0)/revs
            radius = start_radius + (end_radius-start_radius)*pct
            alt = c2.alt + (c3.alt-c2.alt)*pct
            angle = start_angle + (end_angle-start_angle)*pct;
            
            offset_x = radius * cos(angle+distance.DTR(j))
            offset_y = radius * sin(angle+distance.DTR(j))
            offset_x /= earth_radius * cos(distance.DTR(c1.lat))
            offset_y /= earth_radius;
            
            pts.append(Coord(c1.lat+distance.RTD(offset_y),
                c1.lon+distance.RTD(offset_x), alt))
            j += 360/(2*pi*radius/4)
    pts.append(c3)
    return pts
    
def gpxdump(pts):
    gpx = GPX()
    
    # Create track
    gpx_track = GPXTrack(name="Spiral pattern")
    gpx.tracks.append(gpx_track)
    # Create segment in GPX track:
    gpx_segment = GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)
    
    for pt in pts:
        gpx_segment.points.append(GPXTrackPoint(pt.lat,pt.lon,elevation=pt.alt))
        
    with open("spiral.gpx", "w") as fpo:
        print(gpx.to_xml(), file=fpo)
    

    