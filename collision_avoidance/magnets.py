
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

def haversine(loc1, loc2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    loc1.lon, loc1.lat, loc2.lon, loc2.lat = map(radians, [loc1.lon, loc1.lat, loc2.lon, loc2.lat])
    # haversine formula 
    dlon = loc2.lon - loc1.lon 
    dlat = loc2.lat - loc1.lat 
    dalt = abs(loc2.alt - loc1.alt)

    a = sin(dlat/2)**2 + cos(loc1.lat) * cos(loc2.lat) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    km = 6371 * c

    return km

def difference(point1, point2, acs):
    point2_cp = copy.copy(point1)

    if acs(point2_cp) == point1.lat:
        point2_cp.lat = point2.lat
    elif acs(point2_cp) == point1.lon:
        point2_cp.lon = point2.lon
    elif acs(point2_cp) == point1.alt:
        point2_cp.alt = point2.alt

    return distance(point1, point2_cp)


def gradient_1d(acs, drone, waypoint, other_drones, D, C_a, C_r):
    G = C_a * difference(waypoint, drone, acs)

    ''' 
    for d in other_drones:
        dist = distance(d, drone)
        G += C_r * (acs(d) - acs(drone)) / ((1 - dist) ** 3 * dist)
    '''

    return G


def force(drone, waypoint, other_drones, D=1, C_a=1, C_r=5):
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

    return drone.lat + G_x, drone.lon + G_y, drone.alt + G_z

