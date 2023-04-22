import numpy as np
import path
import time
import math 
import psutil
import argparse
import copy 
#from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, LocationGlobalRelative

# define state of the system
# state = [latitude, longitude, altitude, vx, vy, vz, pitch, roll, yaw]
state_dim = 9

# define motion model
# assume constant velocity and small changes in orientation
def motion_model(state, dt):
    # state: [latitude, longitude, altitude, vx, vy, vz, pitch, roll, yaw]
    # dt: time step
    lat, lon, alt, vx, vy, vz, pitch, roll, yaw = state
    p, r, y = np.radians([pitch, roll, yaw])
    x = np.array([lat, lon, alt, vx, vy, vz, p, r, y])
    F = np.eye(state_dim)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    F[3:6, 6:9] = np.array([
        [np.cos(y)*np.cos(p), -np.sin(y)*np.cos(r)+np.cos(y)*np.sin(p)*np.sin(r), np.sin(y)*np.sin(r)+np.cos(y)*np.sin(p)*np.cos(r)],
        [np.sin(y)*np.cos(p), np.cos(y)*np.cos(r)+np.sin(y)*np.sin(p)*np.sin(r), -np.cos(y)*np.sin(r)+np.sin(y)*np.sin(p)*np.cos(r)],
        [-np.sin(p), np.cos(p)*np.sin(r), np.cos(p)*np.cos(r)]
    ])
    Q = np.diag([1e-7, 1e-7, 1e-2, 1e-7, 1e-7, 1e-2, 1e-6, 1e-6, 1e-6])
    x_next = F.dot(x) + np.random.multivariate_normal(np.zeros(state_dim), Q)
    return x_next

# define likelihood function
def likelihood(state, obs):
    # state: [latitude, longitude, altitude, vx, vy, vz, pitch, roll, yaw]
    # obs: [lat_delay, lon_delay, alt_delay, pitch_delay, roll_delay, yaw_delay]
    lat, lon, alt, _, _, _, pitch, roll, yaw = state
    lat_delay, lon_delay, alt_delay, pitch_delay, roll_delay, yaw_delay = obs
    R_e = 6371000  # radius of earth in meters
    lat1, lat2 = np.radians([lat_delay, lat])
    lon1, lon2 = np.radians([lon_delay, lon])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    distance = R_e * c
    bearing = np.degrees(np.arctan2(np.sin(dlon)*np.cos(lat2), np.cos(lat1)*np.sin(lat2)-np.sin(lat1)*np.cos(lat2)*np.cos(dlon)))
    altitude_error = np.abs(alt - alt_delay)
    orientation_error = np.abs(pitch - pitch_delay) + np.abs(roll - roll_delay) + np.abs(yaw - yaw_delay)
        # calculate likelihood as inverse of error
    likelihood = 1.0 / (distance + altitude_error + orientation_error)
    return likelihood

# define particle filter
class ParticleFilter:
    def __init__(self, n_particles, initial_state):
        self.n_particles = n_particles
        self.state_dim = state_dim
        self.particles = np.zeros((n_particles, state_dim))
        self.weights = np.ones(n_particles) / n_particles
        self.initialize_particles(initial_state)

    def initialize_particles(self, initial_state):
        self.particles = np.random.multivariate_normal(initial_state, np.diag([1e-6, 1e-6, 1e-4, 1e-6, 1e-6, 1e-4, 1e-6, 1e-6, 1e-6]), self.n_particles)

    def predict(self, dt):
        for i in range(self.n_particles):
            self.particles[i] = motion_model(self.particles[i], dt)

    def update(self, obs):
        for i in range(self.n_particles):
            self.weights[i] *= likelihood(self.particles[i], obs)
        self.weights /= np.sum(self.weights)
        self.resample()

    def resample(self):
        indices = np.random.choice(np.arange(self.n_particles), size=self.n_particles, replace=True, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.n_particles) / self.n_particles

    def estimate(self):
        mean_state = np.average(self.particles, axis=0, weights=self.weights)
        return mean_state
    

    
# set initial state estimates
latitude_0 = 37.8095466 
longitude_0 = 28.9804745  
altitude_0 = 50  # meters
pitch_0 = 0  # degrees
roll_0 = 0  # degrees
yaw_0 = 0  # degrees

# initialize particle filter
n_particles = 1000
initial_state = [latitude_0, longitude_0, altitude_0, 0, 0, 0, pitch_0, roll_0, yaw_0]
pf = ParticleFilter(n_particles, initial_state)

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
        lat_delay = float(iha.pos_lat)
        lon_delay = float(iha.pos_lon)
        alt_delay = float(iha.pos_alt_rel)
        pitch_delay = float(iha.att_pitch_deg)
        roll_delay = float(iha.att_roll_deg)
        yaw_delay = float(iha.att_heading_deg)

        # predict and update particle filter
        dt = 1  # time step in seconds
        pf.predict(dt)
        obs = [lat_delay, lon_delay, alt_delay, pitch_delay, roll_delay, yaw_delay]
        pf.update(obs)

        # estimate airplane position
        estimated_state = pf.estimate()
        latitude_est, longitude_est, altitude_est, _, _, _, pitch_est, roll_est, yaw_est = estimated_state

        
        # print estimated position and orientation
        print(f'latitude: {latitude_est}, longitude: {longitude_est}, altitude: {altitude_est}')
        print(f'pitch: {pitch_est}, roll: {roll_est}, yaw: {yaw_est}')

        wp1 = LocationGlobalRelative(latitude_est,longitude_est, altitude_est)
        iha2.goto(wp1)

        # wait for next time step
        time.sleep(dt)









"""""
# simulate airplane motion and observations
T = len(lat_delay)
for t in range(T):
    # simulate airplane motion
    dt = 1  # time step in seconds
    pf.predict(dt)
    
    # update particle weights based on observations
    obs = [lat_delay[t], lon_delay[t], alt_delay[t], pitch_delay[t], roll_delay[t], yaw_delay[t]]
    pf.update(obs)

# estimate airplane position
estimated_state = pf.estimate()
latitude_est, longitude_est, altitude_est, _, _, _, pitch_est, roll_est, yaw_est = estimated_state
"""