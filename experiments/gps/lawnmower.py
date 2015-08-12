#!/usr/bin/env python3
import sys,os,re
import distance
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint

class Coord:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
    def __str__(self):
        return "<%.6f, %.6f>" % (self.lat, self.lon)

def lawnmower(lat1, lon1, lat2, lon2):
    '''Good for small distances only.'''
    SWEEP_SPACING = 3 #3m
    #Determine which way we are sweeping
    d1 = distance.haversine(lat1, lon1, lat1, lon2)
    d2 = distance.haversine(lat1, lon2, lat2, lon2)
    points = int(min(d1, d2)//SWEEP_SPACING)
    pts = []
    
    if points != 0:
        modlat = True
        if d1 > d2:
            frac = (lat2 - lat1) / points
        else:
            frac = (lon2 - lon1) / points
            modlat = False
        
        for i in range(points):
            v1 = Coord(lat1 + frac*i, lon1) if modlat else Coord(lat1, lon1 + frac*i)
            v2 = Coord(v1.lat, lon2) if modlat else Coord(lat2, v1.lon)
            if i%2:
                pts.append(v2)
                pts.append(v1)
            else:
                pts.append(v1)
                pts.append(v2)
        if not points%2:
            pts.append(Coord(lat2, lon1) if modlat else Coord(lat1, lon2))
    else:
        pts.append(Coord(lat1, lon1))
    pts.append(Coord(lat2, lon2))
    
    return pts
    
def gpxdump(pts):
    gpx = GPX()
    
    # Create track
    gpx_track = GPXTrack(name="Lawnmower pattern")
    gpx.tracks.append(gpx_track)
    # Create segment in GPX track:
    gpx_segment = GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)
    
    for pt in pts:
        gpx_segment.points.append(GPXTrackPoint(pt.lat,pt.lon))
        
    with open("lawnmower.gpx", "w") as fpo:
        print(gpx.to_xml(), file=fpo)
    

    