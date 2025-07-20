# Power Unit Model

# Insert file path to call UDFs
import csv
file_path = r'C:\Users\Micha\OneDrive - University of Leeds\FS\F25\LTS\PointMassLapSim\Parameters'

# Load all necessary modules
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splrep, splev
from Drivetrain_model import DT_output 
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
# Tyre rolling radius [m]
Rr = 0.2286
# Final drive ratio
final_drive_ratio = 5.5
# PU efficiency
motor_efficiency = 0.85


#------------------------------------------- IC Drivetrain Parameter Processing

def IC_PU_output():
    # Initialise arrays
    engine_speed = []
    bhp = [] 
    
    # Open file and append to bhp and engine speed arrays
    with open(r'C:\Users\Micha\OneDrive - University of Leeds\FS\F25\LTS\PointMassLapSim\Parameters\bhp_rpm_KTM500.csv', mode = 'r') as csvfile:
        data = csv.reader(csvfile)
        for row in data:
            # Append values to the correct arrays
            engine_speed.append(row[0])
            bhp.append(row[1])
            
    # Remove column titles + convert to floats
    engine_speed = engine_speed[1:]
    bhp = bhp[1:]
    
    for i in range(0, len(engine_speed)):
        engine_speed[i] = int(engine_speed[i])
        bhp[i] = int(bhp[i])
    
    # Fit curve to increase resolution of data
    tck = splrep(engine_speed, bhp, s=0)  # tck are the spline parameters,
    # a tuple containing the B-spline coefficients, the vector of knots (coordinates)
    # and the degree of the spline (3rd order). u is an array of the values of 
    # the parameter.
    
    # Define an array over which we want the curve to be fitted - 8500 data points
    engine_speeds_fitted = np.linspace(engine_speed[0], engine_speed[-1], engine_speed[-1])
    # Evaluate the spline at the user-defined points
    bhp_fitted = splev(engine_speeds_fitted, tck)
    # Use splev to evaluate the smoothing polynomial and its derivatives given
    # input knots and coeffs. i.e. take the fitted curve and plot is over the 
    # user defined range, u_fine.
    
    # Plot relationship
    # plt.figure(figsize=(16, 9))
    # plt.scatter(engine_speed, bhp)
    # plt.plot(engine_speeds_fitted, bhp_fitted)
    # plt.title('BHP - Engine Speed Map')
    # plt.xlabel('Engine Speed [rpm]')
    # plt.ylabel('Brake Horse Power [hp]')
    # plt.grid()
    # plt.show()
    
#----------------------------------------------------------------- Wheel Speed
    
    # Load in drivetrain values
    final_drive_ratio, G1_ratio, G2_ratio, G3_ratio, G4_ratio, G5_ratio, G6_ratio, change_time, transmission_efficiency = DT_output()
    
    # Calculate wheel speed for corresponding engine speeds
    wheel_speeds_fitted_1 = []
    wheel_speeds_fitted_1_mph = []
    wheel_speeds_fitted_2 = []
    wheel_speeds_fitted_2_mph = []
    wheel_speeds_fitted_3 = []
    wheel_speeds_fitted_3_mph = []
    wheel_speeds_fitted_4 = []
    wheel_speeds_fitted_4_mph = []
    wheel_speeds_fitted_5 = []
    wheel_speeds_fitted_5_mph = []
    wheel_speeds_fitted_6 = []
    wheel_speeds_fitted_6_mph = []
            
    # Cycle through gears and calculate corresponding wheel speeds
    for i in range(0, 6):
        for j in range(0, len(engine_speeds_fitted)):
            if i == 0:
                # Calculate speed at wheel
                wheel_speed = 2*math.pi*Rr*((engine_speeds_fitted[j])/(60*G1_ratio*final_drive_ratio*transmission_efficiency))
                wheel_speeds_fitted_1.append(wheel_speed)    
                wheel_speeds_fitted_1_mph.append(wheel_speed*2.23694) # Convert to mph
            elif i == 1:
                # Calculate speed at wheel
                wheel_speed = 2*math.pi*Rr*((engine_speeds_fitted[j])/(60*G2_ratio*final_drive_ratio*transmission_efficiency))
                wheel_speeds_fitted_2.append(wheel_speed)  
                wheel_speeds_fitted_2_mph.append(wheel_speed*2.23694) # Convert to mph
            elif i == 2:
                # Calculate speed at wheel
                wheel_speed = 2*math.pi*Rr*((engine_speeds_fitted[j])/(60*G3_ratio*final_drive_ratio*transmission_efficiency))
                wheel_speeds_fitted_3.append(wheel_speed)   
                wheel_speeds_fitted_3_mph.append(wheel_speed*2.23694) # Convert to mph
            elif i == 3:
                # Calculate speed at wheel
                wheel_speed = 2*math.pi*Rr*((engine_speeds_fitted[j])/(60*G4_ratio*final_drive_ratio*transmission_efficiency))
                wheel_speeds_fitted_4.append(wheel_speed)  
                wheel_speeds_fitted_4_mph.append(wheel_speed*2.23694) # Convert to mph
            elif i == 4:
                # Calculate speed at wheel
                wheel_speed = 2*math.pi*Rr*((engine_speeds_fitted[j])/(60*G5_ratio*final_drive_ratio*transmission_efficiency))
                wheel_speeds_fitted_5.append(wheel_speed)
                wheel_speeds_fitted_5_mph.append(wheel_speed*2.23694) # Convert to mph
            else:
                # Calculate speed at wheel
                wheel_speed = 2*math.pi*Rr*((engine_speeds_fitted[j])/(60*G6_ratio*final_drive_ratio*transmission_efficiency))
                wheel_speeds_fitted_6.append(wheel_speed)
                wheel_speeds_fitted_6_mph.append(wheel_speed*2.23694) # Convert to mph
             
    # Plot relationship
    plt.figure(figsize=(16, 9))
    plt.plot(wheel_speeds_fitted_1_mph, bhp_fitted, wheel_speeds_fitted_2_mph, bhp_fitted, wheel_speeds_fitted_3_mph, bhp_fitted, wheel_speeds_fitted_4_mph, bhp_fitted, wheel_speeds_fitted_5_mph, bhp_fitted, wheel_speeds_fitted_6_mph, bhp_fitted)
    plt.title('BHP vs Wheel Speed')
    plt.xlabel('Wheel Speed [mph]')
    plt.ylabel('BHP [hp]')
    plt.grid()
    plt.show()
    
    
    # # Calculate Te (engine torque) for given bhp and engine speed.
    # Te = []
    # for i in range(0, len(engine_speeds_fitted)):
    #     Te.append(bhp_fitted[i]/engine_speeds_fitted[i])
    
    # # Initlialise arrays
    # TE_g1 = []
    # TE_g2 = []
    # TE_g3 = []
    # TE_g4 = []
    # TE_g5 = []
    # TE_g6 = []
    
    # # Cycle through gears and calculate corresponding wheel torques
    # for i in range(0, 6):
    #     for j in range(0, len(Te)):
    #         if i == 0:
    #             # Calculate torque at wheel
    #             Tw_g1 = (Te[j]*G1_ratio*final_drive_ratio*transmission_efficiency)
    #             # Calculate tractive effort
    #             TE_g1.append(Tw_g1/Rr)
            
    #         elif i == 1:
    #             Tw_g2 = (Te[j]*G2_ratio*final_drive_ratio*transmission_efficiency)
    #             TE_g2.append(Tw_g2/Rr)
    #         elif i == 2:
    #             Tw_g3 = (Te[j]*G3_ratio*final_drive_ratio*transmission_efficiency)
    #             TE_g3.append(Tw_g3/Rr)
    #         elif i == 3:
    #             Tw_g4 = (Te[j]*G4_ratio*final_drive_ratio*transmission_efficiency)
    #             TE_g4.append(Tw_g4/Rr)
    #         elif i == 4:
    #             Tw_g5 = (Te[j]*G5_ratio*final_drive_ratio*transmission_efficiency)
    #             TE_g5.append(Tw_g5/Rr)
    #         else:
    #             Tw_g6 = (Te[j]*G6_ratio*final_drive_ratio*transmission_efficiency)
    #             TE_g6.append(Tw_g6/Rr)
    
    # # Plot relationship
    # plt.figure(figsize=(16, 9))
    # plt.plot(engine_speeds_fitted, TE_g1, engine_speeds_fitted, TE_g2, engine_speeds_fitted, TE_g3, engine_speeds_fitted, TE_g4, engine_speeds_fitted, TE_g5, engine_speeds_fitted, TE_g6)
    # plt.title('Tractive Effort vs Engine Speed')
    # plt.xlabel('Engine Speed [rpm]')
    # plt.ylabel('Tractive Effort [N]')
    # plt.grid()
    # plt.show()
    
    # Return 
    # return TE_g1, TE_g2, TE_g3, TE_g4, TE_g5, TE_g6
    
    return wheel_speeds_fitted_1, wheel_speeds_fitted_2, wheel_speeds_fitted_3, wheel_speeds_fitted_4, wheel_speeds_fitted_5, wheel_speeds_fitted_6, bhp_fitted

# def IC_stitch_bhp_wheel_speed(wheel_speeds_fitted_1, wheel_speeds_fitted_2, wheel_speeds_fitted_3, 
#                            wheel_speeds_fitted_4, wheel_speeds_fitted_5, wheel_speeds_fitted_6, 
#                            bhp_fitted):
    
    # # Combine all wheel speeds into one large array
    # all_wheel_speeds = np.concatenate([
    #     wheel_speeds_fitted_1, wheel_speeds_fitted_2, wheel_speeds_fitted_3,
    #     wheel_speeds_fitted_4, wheel_speeds_fitted_5, wheel_speeds_fitted_6
    # ])
    
    # # Create an array to store the BHP corresponding to each wheel speed
    # all_bhp = np.concatenate([bhp_fitted] * 6)  # Same bhp for all gears
    
    # # Sort the combined wheel speeds and their corresponding BHP
    # sorted_indices = np.argsort(all_wheel_speeds)
    # all_wheel_speeds_sorted = all_wheel_speeds[sorted_indices]
    # all_bhp_sorted = all_bhp[sorted_indices]

    # # Initialize a dictionary to store max BHP at each unique wheel speed
    # max_bhp_at_wheel_speed = {}
    
    # # Loop through each gear's wheel speeds and corresponding BHP
    # for gear_idx, wheel_speeds in enumerate([wheel_speeds_fitted_1, wheel_speeds_fitted_2, 
    #                                          wheel_speeds_fitted_3, wheel_speeds_fitted_4, 
    #                                          wheel_speeds_fitted_5, wheel_speeds_fitted_6]):
        
    #     for speed in wheel_speeds:
    #         bhp = bhp_fitted[gear_idx]  # Each gear shares the same BHP data for simplicity
    #         if speed in max_bhp_at_wheel_speed:
    #             max_bhp_at_wheel_speed[speed] = max(max_bhp_at_wheel_speed[speed], bhp)
    #         else:
    #             max_bhp_at_wheel_speed[speed] = bhp
    
    # # Extract unique wheel speeds and their corresponding max BHP
    # unique_wheel_speeds = np.array(list(max_bhp_at_wheel_speed.keys()))
    # max_bhp_values = np.array(list(max_bhp_at_wheel_speed.values()))
    
    # # Sort the results
    # sorted_indices = np.argsort(unique_wheel_speeds)
    # unique_wheel_speeds = unique_wheel_speeds[sorted_indices]
    # max_bhp_values = max_bhp_values[sorted_indices]

    # # Plot the stitched BHP vs Wheel Speed graph
    # plt.figure(figsize=(16, 9))
    # plt.plot(unique_wheel_speeds * 2.23694, max_bhp_values, color='r', label="Optimal BHP Curve")
    # plt.title('Optimal BHP vs Wheel Speed')
    # plt.xlabel('Wheel Speed [mph]')
    # plt.ylabel('BHP [hp]')
    # plt.grid()
    # plt.legend()
    # plt.show()

    # return unique_wheel_speeds, max_bhp_values
    
    #-------------------------------------------------------------- DEVELOPMENT
    # # Wheel speeds and BHP per gear
    # wheel_speeds = [wheel_speeds_gear_1, wheel_speeds_gear_2, wheel_speeds_gear_3,
    #             wheel_speeds_gear_4, wheel_speeds_gear_5, wheel_speeds_gear_6]

    # # Create a continuous plot of BHP vs Wheel Speed
    # plt.figure(figsize=(14, 8))
    
    # # Plot each gear
    # for i, speed in enumerate(wheel_speeds):
    #     plt.plot(speed, bhp_values, label=f'Gear {i+1}')
    
    # # Add plot labels and title
    # plt.xlabel('Wheel Speed [mph]')
    # plt.ylabel('BHP [hp]')
    # plt.title('BHP vs Wheel Speed (Continuous Curve)')
    # plt.grid(True)
    # plt.legend()
    
    # # Display the plot
    # plt.show()
    
#------------------------------------------- EV Drivetrain Parameter Processing

def EV_PU_output():
    # Initialise arrays
    motor_speed = [] # rpm
    continuous_power = [] # kW
    peak_power = [] # kW
    
    # Open file and append to bhp and engine speed arrays
    with open(r'C:\Users\Micha\OneDrive - University of Leeds\FS\F25\LTS\PointMassLapSim\Parameters\EMRAX_228_HV_CC_P_motor_speed.csv', mode = 'r') as csvfile:
        data = csv.reader(csvfile)
        for row in data:
            # Append values to the correct arrays
            motor_speed.append(row[0])
            continuous_power.append(row[1])
            peak_power.append(row[2])
            
    # Remove column titles + convert to floats
    motor_speed = motor_speed[1:]
    continuous_power = continuous_power[1:]
    peak_power = peak_power[1:]
    
    for i in range(0, len(motor_speed)):
        motor_speed[i] = float(motor_speed[i])
        continuous_power[i] = float(continuous_power[i])
        peak_power[i] = float(peak_power[i])        
    
    # Fit curve to increase resolution of data
    tck = splrep(motor_speed, peak_power, s=0)  # tck are the spline parameters,
    # a tuple containing the B-spline coefficients, the vector of knots (coordinates)
    # and the degree of the spline (3rd order). u is an array of the values of 
    # the parameter.
    
    # Define an array over which we want the curve to be fitted - 5000 data points
    motor_speed_fitted = np.linspace(int(motor_speed[0]), int(motor_speed[-1]), int(motor_speed[-1]))
    # Evaluate the spline at the user-defined points
    power_fitted = splev(motor_speed_fitted, tck)
    # Use splev to evaluate the smoothing polynomial and its derivatives given
    # input knots and coeffs. i.e. take the fitted curve and plot is over the 
    # user defined range, u_fine.
    
    # Plot relationship 
    # plt.figure(figsize=(16, 9))
    # plt.scatter(motor_speed, peak_power)
    # plt.plot(motor_speed_fitted, power_fitted)
    # plt.title('Peak Power - Motor Speed Map')
    # plt.xlabel('Motor Speed [rpm]')
    # plt.ylabel('Peak power [kW]')
    # plt.grid()
    # plt.show()
    
    # Calculate wheel speeds
    # Initialise wheel speeds array
    wheel_speeds_fitted = np.zeros([len(motor_speed_fitted)])
    
    for j in range(0, len(motor_speed_fitted)):
        wheel_speed = 2*math.pi*Rr*((motor_speed_fitted[j])/(60*final_drive_ratio*motor_efficiency)) 
        # No gear ratio, only final drive and overall efficiency
        
        wheel_speeds_fitted[j] = wheel_speed # wheel speed in m/s *by
        # 2.236936 for mph
        
    plt.figure(figsize=(16, 9))
    plt.scatter(motor_speed, peak_power)
    plt.plot(motor_speed_fitted, power_fitted)
    plt.title('Peak Power - Wheel Speed Map')
    plt.xlabel('Wheel Speed [m/s]')
    plt.ylabel('Peak power [kW]')
    plt.grid()
    plt.show()
            
    return wheel_speeds_fitted, power_fitted
