import json
import os

def loadConfig(configPath):
    with open(configPath, 'r') as f:
        return json.load(f)

configPath = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config.json'))
config = loadConfig(configPath)


def main():

    # Import modules
    from track.track import loadTrack
    from vehicle.vehicle import createVehicle
    from simulator.simulator import runLapTimeSimulation

    # Load Track data
    print("Using track filepath:", config["track"]["filepath"])
    try: 
        track = loadTrack(config["track"]["filepath"], config.get("debugMode", False))
        print(f"Loaded track with {len(track.points)} points, total length: {track.total_length:.2f} m")

    except Exception as e:
        print("Error loading track:", e)
        return

    # Load vehicle parameters
    try:
        vehicle = createVehicle()
        print(f"Loaded vehicle: {vehicle.params.name}, mass: {vehicle.params.mass} kg")
    except Exception as e:
        print("Error loading vehicle:", e)
        return
    
    # Initialise simulation loop 
    try:
        runLapTimeSimulation(track, vehicle, config)
    except Exception as e:
        print("Error during simulation:", e)
        return
if __name__ == "__main__":
    main()