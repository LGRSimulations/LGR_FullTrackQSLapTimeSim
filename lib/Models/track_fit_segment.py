# Import required modules
import numpy as np
from scipy.interpolate import splprep, splev

# Function for spline interpolation of track coordinates
def spline_track_fit(track_x_coordinates, track_y_coordinates, num_points):
    # Remove NaN values
    valid_indices = ~np.isnan(track_x_coordinates) & ~np.isnan(track_y_coordinates)
    xCoords = np.array(track_x_coordinates)[valid_indices]
    yCoords = np.array(track_y_coordinates)[valid_indices]

    # Parametric spline interpolation
    tck, u = splprep([xCoords, yCoords], s=0)  # tck are the spline parameters,
    # a tuple containing the B-spline coefficients, the vector of knots (coordinates)
    # and the degree of the spline (3rd order). u is an array of the values of 
    # the parameter.
    u_fine = np.linspace(0, 1, num_points)
    # Use splev to evaluate the smoothing polynomial and its derivatives given
    # input knots and coeffs. i.e. take the fitted curve and plot is over the 
    # user defined range, u_fine.
    x_fine, y_fine = splev(u_fine, tck)

    return x_fine, y_fine, num_points

# Function to calculate curvature and radius of curvature through the spline fit
def curvature_and_radius(x, y):
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    # Curvature
    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    
    # Radius of curvature, in case curvature = 0, set radius to infinity
    with np.errstate(divide='ignore', invalid='ignore'):
        radius = np.where(curvature != 0, 1.0 / curvature, np.inf)
        
    # Apply the threshold to filter out excessively large radii (i.e. straights)
    # radius_threshold = 700
    # radius = np.where(radius > radius_threshold, np.inf, radius)
    
    return curvature, radius

# Function to calculate segment length
def distance_calc(xn, xn1, yn, yn1):
    
    # Caulculate x and y deltas
    dx = xn - xn1
    dy = yn - yn1
    
    distance = np.sqrt((dx**2) + (dy**2))

    return distance

#--------------------------------------------------- SEGMENT RADII CALCULATION

# Define the function to take 3 input coordinates and calculate a radius to fit them analytically
def fit_circle(x1, x2, x3, y1, y2, y3):
    
    # Initialise variables to simplify equation
    dx2 = (x2-x1)
    dx3 = (x3-x1)
    dy2 = (y2-y1)
    dy3 = (y3-y1)
    
    # Circle rad calc:
    # Separate denomenators to improve readability
    denominator1 = 2 * ((dx2 * dy3) - (dx3 * dy2))
    denominator2 = 2 * ((dx3 * dy2) - (dx2 * dy3))
    
    # Separate first and second fractions in rad equation to improve readability:
    item1 = (((dx2**2) * dy3) - ((dx3**2) * dy2) + ((dy2**2) * dy3) - ((dy3**2) * dy2)) / denominator1
    item2 = (((dx2**2) * dx3) - ((dx3**2) * dx2) + ((dy2**2) * dx3) - ((dy3**2) * dx2)) / denominator2
    radius = np.sqrt((item1**2) + (item2**2))
    
    # Centre coordinates calc:
    xc = x1 + (((dx2**2) * dy3) - ((dx3**2) * dy2) + ((dy2**2) * dy3) - ((dy3**2) * dy2)) / denominator1
    yc = y1 + (((dx2**2) * dx3) - ((dx3**2) * dx2) + ((dy2**2) * dx3) - ((dy3**2) * dx2)) / denominator2
    
    # Apply the threshold to filter out excessively large radii (i.e. straights)
    # radius_threshold = 700
    # radius = np.where(radius > radius_threshold, np.inf, radius)
    
    return radius, xc, yc
    
    
    



