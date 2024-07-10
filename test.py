import numpy as np

def inverse_kinematics(global_x, global_y, base_x, base_y, L1, L2):
    # Translate coordinates to the local frame
    x = global_x - base_x
    print("x=",x)
    y = global_y - base_y
    print("y=",y)
    
    # Inverse kinematics calculations
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    print("cos=", cos_theta2)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    print("sin=",sin_theta2)
    
    theta2 = np.arctan2(sin_theta2, cos_theta2)
    print("teta=",theta2)
    
    k1 = L1 + L2 * cos_theta2
    k2 = L2 * sin_theta2
    
    theta1 = np.pi/2 + np.arctan2(y, x) + np.arctan2(k2, k1)
    
    return theta1, theta2

# Example usage:
global_x = 364  # End effector position in global coordinates
global_y = 34
base_x = 500  # Base position in global coordinates
base_y = 400
L1 = 190
L2 = 200

theta1, theta2 = inverse_kinematics(global_x, global_y, base_x, base_y, L1, L2)
print(f"Theta 1: {np.degrees(theta1)} degrees")
print(f"Theta 2: {np.degrees(theta2)} degrees")