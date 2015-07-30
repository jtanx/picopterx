import sys,os,re,math
from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint, GPXWaypoint

DTR = lambda x: x * math.pi/180
RTD = lambda x: x * 180/math.pi
RADIUS_OF_EARTH = 6364.963

COORDS = [[-30, 150], [-31, 150], [-31, 151], [-32, 151], [-33, 151], 
          [-34, 151], [-35, 151], [-40, 151], [-50, 151], [-60, 151],
          [-70, 151], [-80, 151]]

def spherical(lat1, lon1, lat2, lon2):
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    term1 = 111.08956 * (dlat + 0.000001)
    term2 = math.cos(DTR(lat1 + (dlat / 2)))
    term3 = (dlon + 0.000001) / (dlat + 0.000001)
    return abs(term1 / math.cos(math.atan(term2 * term3)))
    
def spherical2(lat1, lon1, lat2, lon2):
    lat1 = DTR(lat1)
    lon1 = DTR(lon1)
    lat2 = DTR(lat2)
    lon2 = DTR(lon2)
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    term1 = 111.08956 * (RTD(dlat) + 0.000001)
    term2 = math.cos(lat1 + (dlat / 2)) * (dlon + 1.7453292519943295e-8)
    term3 = dlat + 1.7453292519943295e-8
    return term1 / math.cos(math.atan2(term2, term3))
    
def spherical3(lat1, lon1, lat2, lon2):
    lat1 = DTR(lat1)
    lon1 = DTR(lon1)
    lat2 = DTR(lat2)
    lon2 = DTR(lon2)
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    term1 = 6364.96293596533 * dlat + 0.00011108956
    term2 = math.cos(lat1 + (dlat / 2))
    term3 = (dlon + 1.7453292519943295e-8) / (dlat + 1.7453292519943295e-8)
    return abs(term1 / math.cos(math.atan(term2*term3)))
    
def haversine(lat1, lon1, lat2, lon2):
    lat1 = DTR(lat1)
    lon1 = DTR(lon1)
    lat2 = DTR(lat2)
    lon2 = DTR(lon2)
    '''
    %timeit math.pow(math.sin(0.23), 2)
    1000000 loops, best of 3: 475 ns per loop

    %timeit math.sin(0.23)*math.sin(0.23)
    1000000 loops, best of 3: 330 ns per loop
    '''
    h = (math.sin((lat2-lat1)/2) ** 2) + math.cos(lat1) * math.cos(lat2) * \
        (math.sin((lon2 - lon1)/2) ** 2)
    return 2 * RADIUS_OF_EARTH * 1000 * math.asin(math.sqrt(h))
    
def test_distance():
    print("Spheroidal (degrees)", "Spheroidal (radians)", "Haversine", sep="\t")
    for i in range( len(COORDS)):
        print(spherical(COORDS[0][0],COORDS[0][1],COORDS[i][0],COORDS[i][1]),
              spherical3(COORDS[0][0],COORDS[0][1],COORDS[i][0],COORDS[i][1]),
              haversine(COORDS[0][0],COORDS[0][1],COORDS[i][0],COORDS[i][1])/1000.0,
              sep="\t")

    
def bearing(lat1, lon1, lat2, lon2):
    lat1 = DTR(lat1)
    lon1 = DTR(lon1)
    lat2 = DTR(lat2)
    lon2 = DTR(lon2)
    x = math.sin(lon2 - lon1) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
    return math.atan2(y,x)
    
def cbearing(lat1, lon1, lat2, lon2):
    lat1 = DTR(lat1)
    lon1 = DTR(lon1)
    lat2 = DTR(lat2)
    lon2 = DTR(lon2)
    
    y = math.sin(lon2-lon1)*math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1)
    return math.atan2(y,x)