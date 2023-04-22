import numpy as np

def proportion_data(input_data, input_min, input_max, output_min, output_max):
    output_data = (input_data - input_min) * (output_max - output_min) / (input_max - input_min) + output_min
    return output_data

def pid_controller(x, y, setpoint_x, setpoint_y, kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch):
    
    # Calculate the errors
    error_x = setpoint_x - x
    error_y = setpoint_y - y
    
    # Initialize variables
    integral_x = 0
    integral_y = 0
    derivative_x = 0
    derivative_y = 0
    
    # Compute the integral and derivative terms
    integral_x += error_x
    integral_y += error_y
    integral_x = np.clip(integral_x, -10, 10) # Clamp the integral term to prevent windup
    integral_y = np.clip(integral_y, -10, 10)
    derivative_x = error_x - pid_controller.prev_error_x
    derivative_y = error_y - pid_controller.prev_error_y
    derivative_x = np.clip(derivative_x, -5, 5) # Limit the derivative term to prevent noise amplification
    derivative_y = np.clip(derivative_y, -5, 5)
    
    # Store the current error for next time
    pid_controller.prev_error_x = error_x
    pid_controller.prev_error_y = error_y
    
    # Compute the PID output
    output_roll = kp_roll * error_x + ki_roll * integral_x + kd_roll * derivative_x
    output_pitch = kp_pitch * error_y + ki_pitch * integral_y + kd_pitch * derivative_y

    # Convert the PID output into angles
    #angle_roll = max(min(output_roll, max_angle_roll), -max_angle_roll)
    #angle_pitch = max(min(output_pitch, max_angle_pitch), -max_angle_pitch)

    return output_roll, output_pitch

# Initialize the previous error terms to zero
pid_controller.prev_error_x = 0
pid_controller.prev_error_y = 0