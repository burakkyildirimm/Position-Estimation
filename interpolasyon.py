import math
import path
import time
import psutil
import argparse
import copy 
#from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, LocationGlobalRelative
from matplotlib.animation import FuncAnimation

# function to calculate the distance between two coordinates using the Vincenty formula
def vincenty_distance(lat1, lon1, lat2, lon2):
    return VincentyDistance((lat1, lon1), (lat2, lon2)).meters

# function to calculate the nearest aircraft
def nearest_aircraft(my_lat, my_lon, my_alt, *aircrafts):
    min_distance = np.inf
    nearest_aircraft = None
    for aircraft in aircrafts:
        d = vincenty_distance(my_lat, my_lon, aircraft["lat"], aircraft["lon"])
        if d < min_distance and aircraft["alt"] > my_alt:
            min_distance = d
            nearest_aircraft = aircraft
    return nearest_aircraft

# Define function to calculate distance between two points
def distance(lat1, lon1, lat2, lon2):
    R = 6371 # Radius of the earth in km
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(math.radians(lat1)) \
        * math.cos(math.radians(lat2)) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c # Distance in km
    return d

# Define function to calculate the actual position
def actual_position(lat1, lon1, lat2, lon2, delay):
    # Convert delay to seconds
    delay_secs = delay * 60
    
    # Calculate the distance traveled during the delay
    dist = distance(lat1, lon1, lat2, lon2)
    
    # Calculate the bearing between the two points
    y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
        math.cos(math.radians(lon2 - lon1))
    bearing = math.degrees(math.atan2(y, x))
    
    # Calculate the speed
    speed = dist / delay_secs
    
    # Calculate the new latitude and longitude
    R = 6371 # Radius of the earth in km
    lat3 = math.asin(math.sin(math.radians(lat1)) * math.cos(speed/R) + \
                     math.cos(math.radians(lat1)) * math.sin(speed/R) * \
                     math.cos(math.radians(bearing)))
    lon3 = math.radians(lon1) + math.atan2(math.sin(math.radians(bearing)) * \
                                           math.sin(speed/R) * math.cos(math.radians(lat1)), \
                                           math.cos(speed/R) - math.sin(math.radians(lat1)) * \
                                           math.sin(lat3))
    
    # Convert latitude and longitude to degrees
    lat3 = math.degrees(lat3)
    lon3 = math.degrees(lon3)
    
    return (lat3, lon3)


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

# Example usage
prev_lat = None # Previous latitude
prev_lon = None # Previous longitude
delay = 10 # Delay time in minutes

while True:
    # Get the current location data
    lat = float(iha.pos_lat)
    lon = float(iha.pos_lon)
    
    # If this is the first iteration, set the previous location to the current location
    if prev_lat is None or prev_lon is None:
        prev_lat = lat
        prev_lon = lon
        print("Waiting for next location data...")
        continue
    
    # Calculate the actual position
    lat3 ,lon3 = actual_position(prev_lat, prev_lon, lat, lon, delay)
    print("Actual Lat: ", lat3)
    #print("Actual Lon: ", lon3)
    print("Prev Lat: ", prev_lat)
    wp1 = LocationGlobalRelative(lat3,lon3, iha.pos_alt_rel)
    iha2.goto(wp1)

    inovation = lat3 - prev_lat 
    #print(inovation)
    # Update the previous location to the current location
    prev_lat = lat
    prev_lon = lon

   
    # Wait for next location data
    time.sleep(1)






