
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import random
from math import radians, cos, sin, asin, sqrt

class Point(object):

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def move(self, v_x, v_y, v_z):
        self.x += v_x
        self.y += v_y
        self.z += v_z

    def loc(self):
        return (self.x, self.y, self.z)

def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    km = 6371 * c
    return km

def distance(p1, p2):
    return ( (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2) ** .5 


def gradient_1d(acs, drone, waypoint, other_drones):
    G = C_a * (acs(waypoint) - acs(drone))

    for d in other_drones:
        G += C_r * (acs(d) - acs(drone)) / ((1 - distance(d, drone)) ** 3 * distance(d, drone))

    return G


def force(drone, waypoint, other_drones, D=1, C_a=1, C_r=5):
    G_x = gradient_1d(lambda x: x.x, drone, waypoint, other_drones)
    G_y = gradient_1d(lambda x: x.y, drone, waypoint, other_drones)
    G_z = gradient_1d(lambda x: x.z, drone, waypoint, other_drones)
    
    denom =  (G_x ** 2 + G_y ** 2 + G_z ** 2) ** .5
    G_x = D * G_x / denom
    G_y = D * G_y / denom 
    G_z = D * G_z / denom 

    return G_x, G_y, G_z

