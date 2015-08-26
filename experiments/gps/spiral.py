#!/usr/bin/env python3
import sys,os,re
import distance
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint
from math import cos,sin,pi

class Coord:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
    def __str__(self):
        return "<%.6f, %.6f>" % (self.lat, self.lon)

def spiral(lat1, lon1, lat2, lon2):
    '''Good for small distances only.'''
    radius = distance.haversine(lat1, lon1, lat2, lon2)
    start_angle = distance.bearing(lat1, lon1, lat2, lon2)
    earth_radius = 6378137
    pts = []
    delta = 360/(2*pi*radius/4)
    print(radius, delta, 360/delta)
    
    i = 0
    while i < 360:
        offset_x = radius * cos(start_angle+distance.DTR(i))
        offset_y = radius * sin(start_angle+distance.DTR(i))
        offset_x /= earth_radius * cos(distance.DTR(lat1))
        offset_y /= earth_radius;
        
        pts.append(Coord(lat1+distance.RTD(offset_y), lon1+distance.RTD(offset_x)))
        i += delta
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
        gpx_segment.points.append(GPXTrackPoint(pt.lat,pt.lon))
        
    with open("spiral.gpx", "w") as fpo:
        print(gpx.to_xml(), file=fpo)
    

    