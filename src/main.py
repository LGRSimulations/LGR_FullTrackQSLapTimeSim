import json
import os

def loadConfig(configPath):
    with open(configPath, 'r') as f:
        return json.load(f)

configPath = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config.json'))
config = loadConfig(configPath)


def main():

    # TODO: Load vehicle data and track data, then run simulation

    # Import modules
    from track.track import loadTrack
    from vehicle.vehicle import createVehicle
    # Create results dir

    # Validation tests

    # Load Track data
    print("Using track filepath:", config["track"]["filepath"])
    try: 
        track = loadTrack(config["track"]["filepath"])
        print(f"Loaded track with {len(track.points)} points, total length: {track.total_length:.2f} m")

    except Exception as e:
        print("Error loading track:", e)
        return

    # Load vehicle parameters
    try:
        vehicle = createVehicle()
        print(f"Loaded vehicle: {vehicle.name}, mass: {vehicle.mass} kg")
    except Exception as e:
        print("Error loading vehicle:", e)
        return
    
if __name__ == "__main__":
    main()