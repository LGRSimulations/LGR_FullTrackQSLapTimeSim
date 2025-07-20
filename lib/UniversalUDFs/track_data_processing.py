#Track coordinates read and process to output 2x array of x and y coordinates

#--------------------------- TRACK DATA PROCESSING ---------------------------#
#input file path and name and output the x and y coordinate arrays
def track_data(file_path):
    # Track layout - FSUK endurance event
    # Import plotting functionality
    # Open txt file in read mode
    trackMap = open(file_path, 'r')
    # Read lines
    lines = trackMap.readlines()
    # Remove lines 1 to 12
    lines[:12] = []
    
    # CHECK
    # print(lines)
    
    # Set up x and y coordinates list
    xCoords = []
    yCoords = []
    
    #put data into a workable format
    for item in lines[0:]:
        # Split the line by tabs
        data = item.strip().split("\t")
        
        # Check if there are enough values to unpack
        if len(data) >= 2:
            x, y = data[:2]  # Unpack the first two values
            xCoords.append(float(x))
            yCoords.append(float(y))
        
    return xCoords, yCoords
        



