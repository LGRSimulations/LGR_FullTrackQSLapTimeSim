# Modes which will be swept to calculate velocities in each segment 
# Import functions
import numpy as np
import math

#----------------------------------------------------------- DEFINED CONSTANTS
# Acceleration due to gravity [m/s^2]
g = 9.81
# Vehicle frontal area [m^2]
frontal_area = 1
# Vehicle coeff drag
coeff_drag = 0.5
# Air density (kg/m^3)
air_density = 1.293
# Car mass [kg]
m = 287
# Car road friction coefficient
car_road_friction_coefficient = 1.4

#------------------------------------------------------------ MAXIMUM VELOCITY 
# Determines maximum theoretical corner speed of a given radius limited by grip
def max_corner_velocity(corner_radius):
    # Split into numerator and denomenator for ease of reading
    # Calculate reaction force at the contact patch - IGNORING DOWNFORCE EFFECTS
    R = m*g
    # Numerator calculation
    numerator = (car_road_friction_coefficient*R)**2
    
    # Denomenator calculation
    denomenator = (m**2/corner_radius**2)+(0.25*(air_density**2)*(frontal_area**2)
                                          *coeff_drag**2)
    
    v_max = (numerator/denomenator)**0.25
    
    return v_max

#------------------------------------------------------ ACCELERATION FROM REST 
# Determines exit speed of a straight sector limited by engine power
def straight_exit_speed(entry_speed, sector_length, wheel_speed, P):
    # Sector entry speed = u, sector length is s, wheel speed array and corresponding motor power output array 
    u = entry_speed
    s = sector_length
    
    # Find the closest value in the wheel_speed array to the entry speed (u)
    closest_index = (abs(wheel_speed - u)).argmin()  # Finds index of closest value
    # closest_wheel_speed = wheel_speed[closest_index]
    
    # Calculate attribute within the square root for clarity
    # print(P[closest_index])
    x = (u**2)+((2*s*(P[closest_index]/u)-(0.5*air_density*(u**2)*frontal_area*coeff_drag))/m)
                                              
    v_exit = (x)**0.5
    
    return v_exit
    
#----------------------------------------------------- CORNER TRANSITION SPEED 
# Determines the speed of the car when transitioning into a corner - throttle 
# requires modulation to stay on track -a method to calculate how much of the force
# available to accelerate the car can be transmitted through the tyres, 
# in order that we can calculate the speed at the end of any track sector.
def corner_transition_speed(n_exit_speed, max_corner_speed):
    # Conduct checks
    if n_exit_speed <= max_corner_speed:
        # Don't change anything
        n_exit_speed = n_exit_speed
        
    else: # i.e. if n_exit_speed > max_corner_speed, set equal to max speed
        n_exit_speed = max_corner_speed
        
    return n_exit_speed
    
#----------------------------------------------------- BRAKING CORRECTED SPEED 
# Determines the speed of the car when taking braking effects into account.
def braking_corrected_speed(n_original_exit_speed, n_original_entry_speed,
                            n_1_braked_entry_speed, corner_radius, segment_length):
    
    # print(n_1_braked_entry_speed, n_original_exit_speed)
    
    # Conduct the flowchart to determine segment n exit speed (= n+1 entry speed)
    # Identify whether braking is necessary
    if n_1_braked_entry_speed < n_original_exit_speed: # then braking required
        
        # Define our variables
        v = n_1_braked_entry_speed
        r = corner_radius
        cd = coeff_drag
        rho = air_density
        A = frontal_area
        s = segment_length
        
        # Calculate reaction force at the contact patch - IGNORING DOWNFORCE EFFECTS
        R = m * g
        
        # Available braking force, Fb (grip limited assumption)
        Fb = math.sqrt((car_road_friction_coefficient**2 * R**2) + ((m**2 * v**4)/r**2))
       
        # Drag force, Fd
        Fd = 0.5 * (cd * rho * v**2 * A)
       
        # Total decelerative force, Fs
        Fs = Fb + Fd
        
        # Total deceleration a
        a = (Fs/m)
        
        # Resulting grip limited segment entry speed for n
        # print(v**2 + (2*s * a))
        max_entry_speed = math.sqrt(v**2 + (2*s * a))
        # print(max_entry_speed)
        # Check what degree of braking is required
        if max_entry_speed > n_original_entry_speed:
             # Transition into braking
             n_entry_speed = n_original_entry_speed
             n_exit_speed = n_1_braked_entry_speed
             
        else: # i.e. if n_original_entry_speed <= max_entry_speed, maximum braking
             n_entry_speed = max_entry_speed
             n_exit_speed = n_1_braked_entry_speed
            
            
    else: # i.e. if n_1_braked_entry_speed <=  n_original_exit_speed do nothing - no braking required
        # Do nothing: new entry speed (n) = original entry speed (n)
        # new exit speed (n) = original exit speed (n)
        n_entry_speed = n_original_entry_speed
        n_exit_speed = n_original_exit_speed
        
 
    return n_entry_speed, n_exit_speed

#--------------------------------------------------- BRAKING CORRECTED SPEED 2 
# Determines the speed of the car when taking braking effects into account.
def braking_corrected_speed2(n_original_exit_speed, n_original_entry_speed,
                            n_1_braked_entry_speed, corner_radius, segment_length):
    
    # Define our variables
    v = n_1_braked_entry_speed
    r = corner_radius
    cd = coeff_drag
    rho = air_density
    A = frontal_area
    s = segment_length
    
    # Calculate reaction force at the contact patch - IGNORING DOWNFORCE EFFECTS
    R = m * g
    
    # Available braking force, Fb (grip limited assumption)
    Fb = math.sqrt((car_road_friction_coefficient**2 * R**2) + ((m**2 * v**4)/r**2))
   
    # Drag force, Fd
    Fd = 0.5 * (cd * rho * v**2 * A)
   
    # Total decelerative force, Fs
    Fs = Fb + Fd
    
    # Total deceleration a
    a = (Fs/m)
    
    # Resulting grip limited segment entry speed for n
    # print(v**2 + (2*s * a))
    max_entry_speed = math.sqrt(v**2 + (2*s * a))
    print(n_1_braked_entry_speed, n_original_exit_speed,max_entry_speed)
    # Conduct the flowchart to determine segment n exit speed (= n+1 entry speed)
    # Identify whether braking is necessary
    if n_1_braked_entry_speed < n_original_exit_speed: # then braking required
        
        # Check what degree of braking is required
        if max_entry_speed > n_original_entry_speed:
             # Transition into braking
             n_entry_speed = n_original_entry_speed
             n_exit_speed = n_1_braked_entry_speed
             
        else: # i.e. if n_original_entry_speed <= max_entry_speed, maximum braking
             n_entry_speed = max_entry_speed
             n_exit_speed = n_1_braked_entry_speed
            
            
    else: # i.e. if n_1_braked_entry_speed <=  n_original_exit_speed do nothing - no braking required
        # Do nothing: new entry speed (n) = original entry speed (n)
        # new exit speed (n) = original exit speed (n)
        n_entry_speed = n_original_entry_speed
        n_exit_speed = n_original_exit_speed
        
 
    return n_entry_speed, n_exit_speed
    
        



