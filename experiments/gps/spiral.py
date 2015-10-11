#!/usr/bin/env python3
import sys,os,re
import distance
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint
from math import cos,sin,pi,ceil

import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.animation as animation

MAX_ANIM=40

class Coord:
    def __init__(self, lat, lon, alt=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
    def __str__(self):
        return "<%.6f, %.6f, %.1f>" % (self.lat, self.lon, self.alt)

def update_lines(num, data, line) :
    l = int(len(data)*(num/MAX_ANIM))
    if l > 0:
        line.set_data(data[:l,0], data[:l,1])
        line.set_3d_properties(data[:l, 2])
    return line
        
def spiral(c1, c2, c3=None):
    '''Good for small distances only.'''
    if c3 is None:
        c3=c2 #Degenerates to a circle pattern
    
    data = []
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
            data.append([offset_x, offset_y, alt])
            offset_x /= earth_radius * cos(distance.DTR(c1.lat))
            offset_y /= earth_radius;
            
            pts.append(Coord(c1.lat+distance.RTD(offset_y),
                c1.lon+distance.RTD(offset_x), alt))
            j += 360/(2*pi*radius/4)
    pts.append(c3)
    
    data = np.array(data)
    fig = plt.figure()
    ax = Axes3D(fig)
    # Setting the axes properties
    ax.set_xlim3d([data[:,0].min(), data[:,0].max()])
    ax.set_xlabel('Offset (m)')
    ax.set_ylim3d([data[:,1].min(), data[:,1].max()])
    #ax.set_ylabel('Y')
    ax.set_zlim3d([data[:,2].min(), data[:,2].max()])
    ax.set_zlabel('Altitude (m)')
    
    line = ax.plot(data[:1,0], data[:1,1], data[:1,2])[0]
    plt.title("Spiral search pattern")
    line_ani = animation.FuncAnimation(fig, update_lines, MAX_ANIM, fargs=(data, line),
                              interval=10, blit=False)
    plt.show()
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
    

    