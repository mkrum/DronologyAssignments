
import numpy as np
import random
from math import radians, cos, sin, asin, sqrt
import copy


def distance(loc1, loc2):

    x1 = loc1.alt * cos(loc1.lat) * sin(loc1.lon)
    y1 = loc1.alt * sin(loc1.lat)
    z1 = loc1.alt * cos(loc1.lat) * cos(loc1.lon)

    x2 = loc2.alt * cos(loc2.lat) * sin(loc2.lon)
    y2 = loc2.alt * sin(loc2.lat)
    z2 = loc2.alt * cos(loc2.lat) * cos(loc2.lon)

    return ( (x1 - x2) **2 + (y1 - y2) **2 + (z1 - z2) ** 2 ) ** .5

def distance_v2(loc1, loc2):
    lat1 = radians(loc1.lat)
    lon1 = radians(loc1.lon)
    alt1 = loc1.alt
 
    lat2 = radians(loc2.lat)
    lon2 = radians(loc2.lon)
    alt2 = loc2.alt
 
    a = 6378137.0 # equatorial radius of the Earth in meters
    b = 6356752.3 # polar radius of the Earth in meters
 
    N1 =  (a**2)/sqrt((a**2 * (cos(lat1)**2)) + (b**2 * (sin(lat1)**2)))
    N2 =  (a**2)/sqrt((a**2 * (cos(lat2)**2)) + (b**2 * (sin(lat2)**2)))
 
    # convert geodetic coordinates to Cartesian coordinates
    x1 = (N1 + alt1)*cos(lat1)*cos(lon1)
    y1 = (N1 + alt1)*cos(lat1)*sin(lon1)
    z1 = (((b**2)/(a**2))*N1 + alt1)*sin(lat1)
 
    x2 = (N2 + alt2)*cos(lat2)*cos(lon2)
    y2 = (N2 + alt2)*cos(lat2)*sin(lon2)
    z2 = (((b**2)/(a**2))*N2 + alt2)*sin(lat2)

    # calculate distance between the two points
    distance_in_meters = sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

    #print(distance_in_meters)
 
    return distance_in_meters

def difference(point1, point2, acs):
    point2_cp = copy.copy(point1)

    if acs(point2_cp) == point1.lat:
        point2_cp.lat = point2.lat
        dif = int(point1.lat > point2.lat)
    elif acs(point2_cp) == point1.lon:
        point2_cp.lon = point2.lon
        dif = int(point1.lon > point2.lon)
    elif acs(point2_cp) == point1.alt:
        point2_cp.alt = point2.alt
        dif = int(point1.alt > point2.alt)

    return dif * distance_v2(point1, point2_cp) + -1 * (1 - dif) * distance_v2(point1, point2_cp)


def gradient_1d(acs, drone, waypoint, other_drones, D, C_a, C_r):
    G = C_a * difference(waypoint, drone, acs)
    
    '''
    for d in other_drones:
        dist = distance_v2(d, drone)
        G += C_r * (acs(d) - acs(drone)) / ((1 - dist) ** 3 * dist)
    '''
    
    return G


def force(drone, waypoint, other_drones, D=1, C_a=1, C_r=25):
    drone = drone.location.global_relative_frame

    other_drones = [ od.location.global_relative_frame for od in other_drones ]

    G_x = gradient_1d(lambda x: x.lat, 
                      drone, waypoint, other_drones, D, C_a, C_r)

    G_y = gradient_1d(lambda x: x.lon, 
                      drone, waypoint, other_drones, D, C_a, C_r)

    G_z = gradient_1d(lambda x: x.alt, 
                      drone, waypoint, other_drones, D, C_a, C_r)
    
    denom =  (G_x ** 2 + G_y ** 2 + G_z ** 2) ** .5
    G_x = D * G_x / denom
    G_y = D * G_y / denom 
    G_z = D * G_z / denom 

    #print(str((G_x)) + " " +  str((G_y)) + " " + str(G_z))
    return drone.lat + (G_x), drone.lon + (G_y), drone.alt + G_z

