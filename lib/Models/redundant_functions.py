# Import required modules
import numpy as np
from scipy.interpolate import splprep, splev

# Function to calculate cumulative arc length of the spline
def cumulative_arc_length(x, y):
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    cumulative_distance = np.cumsum(distances)
    cumulative_distance = np.insert(cumulative_distance, 0, 0)  # Add initial point
    return cumulative_distance

# Function to segment the spline based on cumulative arc length
def segment_spline(x_fine, y_fine, num_segments):
    circuit_length = cumulative_arc_length(x_fine, y_fine) # Calculate array of
    # full lap distance
    total_length = circuit_length[-1] # Find final value of lap length
    segment_lengths = np.linspace(0, total_length, num_segments + 1) # calculate 
    # the segment boundaries

    segment_x = []
    segment_y = []
    for length in segment_lengths:
        index = np.argmax(circuit_length >= length) # Return the index in the 
        # circuit length dataset that is closest to the 'segment length' being
        # evaluated.
        
        # Subsequently, append the corresponding spline coordinates into their
        # respective x, y coordinate arrays.
        segment_x.append(x_fine[index])
        segment_y.append(y_fine[index])
        
    return np.array(segment_x), np.array(segment_y) # These arrays represent 
    # the segment boundaries on the spline.

#Corner curvature radius function

#----------- DELCARE SEGMENT RADII FUNCTION------------#

# Define the function to take 3 input coordinates and calculate a radius to fit them analytically
def fit_circle(x1, x2, x3, y1, y2, y3):
    #initialise variables to simplify equation

    dx2 = (x2-x1)
    dx3 = (x3-x1)
    dy2 = (y2-y1)
    dy3 = (y3-y1)
    
    #circle rad calc:
    #separate denomenators to improve readability
    denominator1 = 2 * ((dx2 * dy3) - (dx3 * dy2))
    denominator2 = 2 * ((dx3 * dy2) - (dx2 * dy3))
    
    #separate first and second fractions in rad equation to improve readability:
    item1 = (((dx2**2) * dy3) - ((dx3**2) * dy2) + ((dy2**2) * dy3) - ((dy3**2) * dy2)) / denominator1
    item2 = (((dx2**2) * dx3) - ((dx3**2) * dx2) + ((dy2**2) * dx3) - ((dy3**2) * dx2)) / denominator2
    curvatureRad = np.sqrt((item1**2) + (item2**2))
    
    #centre coordinates calc:
    xc = x1 + (((dx2**2) * dy3) - ((dx3**2) * dy2) + ((dy2**2) * dy3) - ((dy3**2) * dy2)) / denominator1
    yc = y1 + (((dx2**2) * dx3) - ((dx3**2) * dx2) + ((dy2**2) * dx3) - ((dy3**2) * dx2)) / denominator2
    
    return curvatureRad, xc, yc

# #example - TEST - WORKS AS INTENDED
# x1 = -267.7320452
# x2 = -278.629571
# x3 = -284.668072

# y1 = -848.6222413
# y2 = -849.3296333
# y3 = -848.9758133

# curvatureRad, xc, yc = fit_circle(x1, x2, x3, y1, y2, y3)
# print(curvatureRad, xc, yc)

# plt.scatter((x1, x2, x3), (y1, y2, y3))

# circle = Circle((xc, yc), curvatureRad, color='b', fill=False)
# plt.gca().add_patch(circle)
# plt.gca().axis('equal')
# plt.show()
        

#define the function to take arrays of x and y coordinates associated with 
#segment points to output x centre, y centre and curvature radii arrays
def circle_arrays(segment_x_coords, segment_y_coords):
    #Loop to fit circles to segment points and calculate radius of curvature
    #Initialise curvature radii array and circle centre coordinates
    curvatureRadii = []
    xCentre = []
    yCentre = []
    #initialise indices for which corner rads are calculated - every second coordinate
    # cornerRadSegments = np.linspace(0, len(segmentPoints))
    for i in range(len(segment_x_coords) - 2):
        x_local = segment_x_coords[i:i+3]
        y_local = segment_y_coords[i:i+3]
        curvatureRad, xc, yc = fit_circle(x_local[0], x_local[1], x_local[2], y_local[0], y_local[1], y_local[2])
        #append to list of rads and circle centre coordinates
        curvatureRadii.append(curvatureRad)
        xCentre.append(xc)
        yCentre.append(yc) 
        
    return xCentre, yCentre, curvatureRadii

#---------------SEGMENT THE TRACK--------------#

# Define function to take in an array of x and y trackmap coordinates and find 
# segment coordinates depending on defined number of segments
def segment_coordinates(track_x_coordinates, track_y_coordinates, number_of_segments):

    #Remove NaN values
    valid_indices = ~np.isnan(track_x_coordinates) & ~np.isnan(track_y_coordinates)
    xCoords = np.array(track_x_coordinates)[valid_indices]
    yCoords = np.array(track_y_coordinates)[valid_indices]
    
    #Calculate Cumulative Distance
    # Calculate distances between consecutive points
    distances = np.sqrt(np.diff(xCoords)**2 + np.diff(yCoords)**2)
    
    # Calculate cumulative distance along the racetrack
    cumulative_distance = np.cumsum(distances)
      # Add initial distance as 0
    cumulative_distance = np.insert(cumulative_distance, 0, 0)
    
    #Total length
    totalLength = cumulative_distance[-1]
    
    #Segmentation:
        
    # Ensure segment_distances stay within the range
    segmentLengths = np.linspace(0, totalLength, (number_of_segments) + 1)
    
    #Initialise list of segment point
    segmentPoints = []
    
    # Find the points corresponding to segment divisions
    for length in segmentLengths:
        index = np.argmax(cumulative_distance >= length)
        segmentPoints.append((xCoords[index], yCoords[index]))        
        
    #loop through segmentPoints to gain x and y data independently
    #initialise x and y arrays
    xSegments=[]
    ySegments=[]
    for i in segmentPoints:
        xSegments.append(i[0])
        ySegments.append(i[-1])
    
    #remove duplicated points
    #convert to lists  to allow use in for loop
    xSegments = list(xSegments)
    ySegments = list(ySegments)
    
    i = 0
    while i < len(xSegments) - 1:
        if np.allclose(xSegments[i], xSegments[i+1]) and np.allclose(ySegments[i], ySegments[i+1]):
            del xSegments[i]
            del ySegments[i]
        else:
            i += 1
            
    # Convert back to numpy arrays if needed
    xSegments = np.array(xSegments)
    ySegments = np.array(ySegments)
    
    return xSegments, ySegments
