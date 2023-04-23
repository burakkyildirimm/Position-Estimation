import numpy as np

# Define state variables
x = 0.0  # x-position
y = 0.0  # y-position
z = 0.0  # z-position
vx = 0.0  # x-velocity
vy = 0.0  # y-velocity
vz = 0.0  # z-velocity
phi = 0.0  # roll angle
theta = 0.0  # pitch angle
psi = 0.0  # yaw angle
state = (x, y, z, vx, vy, vz, phi, theta, psi)

# Define input variables
fx = 0.0  # x-force
fy = 0.0  # y-force
fz = 0.0  # z-force
tau_x = 0.0  # x-torque
tau_y = 0.0  # y-torque
tau_z = 0.0  # z-torque
input = (fx, fy, fz, tau_x, tau_y, tau_z)

# Define parameters
m = 1.0  # mass
g = 9.81  # gravitational acceleration
Ixx = 0.1  # moment of inertia about x-axis
Iyy = 0.1  # moment of inertia about y-axis
Izz = 0.1  # moment of inertia about z-axis
params = (m, g, Ixx, Iyy, Izz)

# Define aerodynamic coefficients
Cx = 0.1  # drag coefficient in x-direction
Cy = 0.1  # drag coefficient in y-direction
Cz = 0.1  # drag coefficient in z-direction
Cl = 0.1  # roll moment coefficient
Cm = 0.1  # pitch moment coefficient
Cn = 0.1  # yaw moment coefficient
aerodynamics = (Cx, Cy, Cz, Cl, Cm, Cn)

def uav_dynamic_model(state, input, params, aerodynamics):
    # Unpack state variables
    x, y, z, vx, vy, vz, phi, theta, psi = state
    
    # Unpack input variables
    fx, fy, fz, tau_x, tau_y, tau_z = input
    
    # Unpack parameters
    m, g, Ixx, Iyy, Izz = params
    
    # Unpack aerodynamic coefficients
    Cx, Cy, Cz, Cl, Cm, Cn = aerodynamics
    
    # Compute airspeed and angle of attack
    V = np.sqrt(vx**2 + vy**2 + vz**2)
    alpha = np.arctan2(vz, np.sqrt(vx**2 + vy**2))
    
    # Compute derivatives of state variables
    dx = vx
    dy = vy
    dz = vz
    dvx = (fx + Cx * V**2 * np.sin(alpha)) / m
    dvy = (fy + Cy * V**2 * np.sin(alpha)) / m
    dvz = (fz + Cz * V**2 * np.cos(alpha) - m * g) / m
    dphi = phi_dot(phi, theta, psi) 
    dtheta = theta_dot(phi, theta, psi, V, alpha, Cl, Cm, Cn, Iyy, Izz)
    dpsi = psi_dot(phi, theta, psi, V, alpha, Cl, Cm, Cn, Ixx, Iyy)
    
    # Return state derivatives as a tuple
    return (dx, dy, dz, dvx, dvy, dvz, dphi, dtheta, dpsi)

def phi_dot(phi, theta, psi):
    return np.cos(theta) * np.sin(psi) * (np.tan(theta) * np.sin(phi) + np.cos(phi))

def theta_dot(phi, theta, psi, V, alpha, Cl, Cm, Cn, Iyy, Izz):
    p, q, r = pqr(phi, theta, psi, V, alpha, Cl, Cm, Cn, Iyy, Izz)
    return q + np.sin(phi) * np.tan(theta) * r

def psi_dot(phi, theta, psi, V, alpha, Cl, Cm, Cn, Ixx, Iyy):
    p, q, r = pqr(phi, theta, psi, V, alpha, Cl, Cm, Cn, Iyy, Izz)
    return r - np.cos(phi) * np.tan(theta) * q

def pqr(phi, theta, psi, V, alpha, Cl, Cm, Cn, Iyy, Izz):
    p = (Iyy - Izz) / Ixx * theta_dot(phi, theta, psi, V, alpha, Cl, Cm, Cn, Iyy, Izz)
    q = (Izz - Ixx) / Iyy * phi_dot(phi, theta, psi) + Cm / Iyy * V**2 * np.cos(alpha)
    r = (Ixx - Iyy) / Izz * psi_dot(phi, theta, psi, V, alpha, Cl, Cm, Cn, Ixx, Iyy) + Cn / Izz * V**2 * np.cos(alpha)
    return p, q, r
