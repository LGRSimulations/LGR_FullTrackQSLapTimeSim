# function to calculate 'precursor speeds'

#import required libraries
import numpy as np

#define function to calculate exit speeds for each sector

def vExit (vEntry, longitudinalAcc, deltaX):
    
    #longitudinal acceleration to be obtained from a lookup table
    #assumed constant within a sector.
    #Based on this assumption, s=d/t, we can use suvat equation to calculate
    #exit speed with the following method:
    
    #initialise vExit variable
    vExit = 0
    vExit = vEntry + (longitudinalAcc*(deltaX/vEntry))
    
    return vExit

#define function to calculate maximum cornerning speed for a given sector

def vExit (vEntry, longitudinalAcc, deltaX):
    
    #longitudinal acceleration to be obtained from a lookup table
    #assumed constant within a sector.
    #Based on this assumption, s=d/t, we can use suvat equation to calculate
    #exit speed with the following method:
    
    #initialise vExit variable
    vExit = 0
    vExit = vEntry + (longitudinalAcc*(deltaX/vEntry))
    
    return vExit