# #Plotting functions

# #----------- PLOT ORIGINAL TRACK MAP AND SEGMENT LIMIT POINTS AND CIRCLE RADII ------------#
# #import required libraries
# import matplotlib.pyplot as plt

# #Define function to input track x and y coordinates, segment x and y coordinates,
# #circle centre x and y coordinates and corresponding curvature radii
# #and plot these superimposed on each other
# def track_segment_plot(track_x_coordinates, track_y_coordinates, segment_x_coords,
#                        segment_y_coords, x_centre_coordinates, y_centre_coordinates,
#                        curvature_radii):
#     #set plot aspect ratio
#     plt.figure(figsize=(16, 9))
    
#     plt.plot(track_x_coordinates, track_y_coordinates, label='Original Track')
#     plt.scatter(segment_x_coords, segment_y_coords, color='r', label='Segment Points', s=5)

#     # Plot circles representing the calculated radii
#     #import circle plotting function
#     from matplotlib.patches import Circle
    
#     #plot circles corresponding to each segment to check its calculation.
#     for xc, yc, radius in zip(x_centre_coordinates, y_centre_coordinates, 
#         curvature_radii):
        
#         circle = Circle((xc, yc), radius, color='b', fill=False)
#         plt.gca().add_artist(circle)
    
#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.title('Segmented Racetrack')
#     plt.legend()
#     plt.grid(True)
#     plt.axis('equal')
#     plt.show()
    
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

# Define the function to plot the original track and segments, now with arcs instead of full circles
def track_segment_plot(track_x_coordinates, track_y_coordinates, segment_x_coords,
                       segment_y_coords, x_centre_coordinates, y_centre_coordinates,
                       curvature_radii):
    plt.figure(figsize=(16, 9))
    
    # Plot the original track
    # plt.plot(track_x_coordinates, track_y_coordinates, label='Original Track')
    plt.scatter(segment_x_coords, segment_y_coords, color='r', label='Segment Points', s=5)

    # Plot arcs corresponding to the calculated radii
    for i in range(len(x_centre_coordinates)):
        # Extract center, radius, and the segment points
        xc, yc = x_centre_coordinates[i], y_centre_coordinates[i]
        radius = curvature_radii[i]

        # Define the three points that define the arc
        x_local = segment_x_coords[i:i+3]
        y_local = segment_y_coords[i:i+3]
        
        # Calculate the angles (in degrees) of the points relative to the center of the circle
        angles = [np.degrees(np.arctan2(y_local[j] - yc, x_local[j] - xc)) for j in range(3)]
        
        # Ensure angles are between 0 and 360 degrees
        angles = [(angle + 360) % 360 for angle in angles]
        
        # Determine start and end angles for the arc
        start_angle = min(angles)
        end_angle = max(angles)
        
        # Handle case where the arc crosses the 360/0 degree boundary
        if end_angle - start_angle > 180:
            start_angle, end_angle = end_angle, start_angle

        # Create and add the arc
        arc = Arc((xc, yc), 2*radius, 2*radius, angle=0,
                  theta1=start_angle, theta2=end_angle, color='#00A19B')
        plt.gca().add_patch(arc)
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim(-450, -225)
    plt.ylim(-900, -450)
    plt.title('Segmented Racetrack with Curvature Arcs')
    plt.legend()
    plt.grid(True)
    plt.show()







