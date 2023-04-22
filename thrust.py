import math

# Constants
kp = 0.5
ki = 0.1
kd = 0.2
c1 = 0.5
c2 = 0.2

# Initialize variables
dt = 0.01
error_sum = 0
last_error = 0
target_speed = 100
current_speed = 50
coefficients = [0.1, 0.2, 0.3]

while True:
    # Compute error and update error sum
    error = target_speed - current_speed
    error_sum += error * dt
    
    # Compute derivative of error
    error_diff = (error - last_error) / dt
    last_error = error
    
    # Compute control output
    control_output = kp * error + ki * error_sum + kd * error_diff
    
    # Compute aerodynamic coefficients
    aoa = math.atan2(coefficients[1], coefficients[0])
    lift_coefficient = c1 * math.sin(2 * aoa) + c2 * math.cos(aoa)
    drag_coefficient = c1 * math.cos(aoa) - c2 * math.sin(2 * aoa)
    
    # Compute thrust required to maintain speed
    thrust = 0.5 * lift_coefficient * coefficients[2] * current_speed ** 2 + drag_coefficient * coefficients[2] * current_speed
    
    # Adjust speed based on control output and thrust
    acceleration = (thrust - control_output) / coefficients[2]
    current_speed += acceleration * dt
    
    # Print current speed and control output
    print("Current speed:", current_speed)
    print("Control output:", control_output)
