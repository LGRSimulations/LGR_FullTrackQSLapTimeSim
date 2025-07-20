# Drivetrain Model

# Insert file path to call UDFs
import csv
file_path = r'C:\Users\Micha\OneDrive - University of Leeds\FS\F25\LTS\PointMassLapSim\Parameters'

# Load all necessary modules
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splrep, splev

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
# Rolling radius
Rr = 0.2286

#--------------------------------------------- Drivetrain Parameter Processing

def DT_output():
    # Initialise arrays
    parameters = []
    values = [] 
    
    # Open file and append to bhp and engine speed arrays
    with open(r'C:\Users\Micha\OneDrive - University of Leeds\FS\F25\LTS\PointMassLapSim\Parameters\drivetrain_parameters_KTM500.csv', mode = 'r') as csvfile:
        data = csv.reader(csvfile)
        for row in data:
            # Append values to the correct arrays
            parameters.append(row[0])
            values.append(row[1])
            
    # Remove column titles + convert to floats
    parameters = parameters[1:]
    values = values[1:]
    
    final_drive_ratio = float(values[0])
    G1_ratio = float(values[1])
    G2_ratio = float(values[2])
    G3_ratio = float(values[3])
    G4_ratio = float(values[4])
    G5_ratio = float(values[5])
    G6_ratio = float(values[6])
    change_time = float(values[7])
    transmission_efficiency = float(values[8])
    
    # Return bhp - engine speed relationship
    return final_drive_ratio, G1_ratio, G2_ratio, G3_ratio, G4_ratio, G5_ratio, G6_ratio, change_time, transmission_efficiency




    
    
        



