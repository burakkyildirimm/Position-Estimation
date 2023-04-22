import math
import path 
import numpy as np
import time
import psutil
import argparse
import copy 
#from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, LocationGlobalRelative


R = 6371000  # Earth's radius in meters


def basic_calculate(curr_lat, curr_lon,curr_yaw, speed, delay):
    
    # Convert lat/lon to radians
    lat_rad = math.radians(curr_lat)
    lon_rad = math.radians(curr_lon)

    # Calculate the distance travelled in the delay period
    distance = delay * speed # assume a speed of 100 meters per second

    # Calculate the bearing based on the initial and current orientations
    bearing = math.radians(curr_yaw)

    # Calculate the new latitude and longitude based on the distance travelled and bearing
    new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(distance/R) + math.cos(lat_rad) * math.sin(distance/R) * math.cos(bearing))
    new_lon_rad = lon_rad + math.atan2(math.sin(bearing) * math.sin(distance/R) * math.cos(lat_rad), math.cos(distance/R) - math.sin(lat_rad) * math.sin(new_lat_rad))

    # Convert new lat/lon to degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)

    return  new_lat, new_lon

# connect sitl
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='tcp:127.0.0.1:5773')
args = parser.parse_args()

connection_string = args.connect

iha = path.Plane(connection_string)

# sitl 2
# connect sitl
parser1 = argparse.ArgumentParser()
parser1.add_argument('--connect', default='tcp:127.0.0.1:5763')
args1 = parser1.parse_args()

connection_string1 = args1.connect

iha2 = path.Plane(connection_string1)

while True:
    lat_est, lon_est = basic_calculate(iha.pos_lat,iha.pos_lon,iha.att_heading_deg,iha.groundspeed,2)

    wp1 = LocationGlobalRelative(lat_est,lon_est, iha.pos_alt_rel)
    iha2.goto(wp1)
    print(lat_est,lon_est)

