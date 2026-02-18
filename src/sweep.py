import json
import os

def load_config(config_path):
    with open(config_path, 'r') as f:
        return json.load(f)

config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config.json'))
config = load_config(config_path)


def main():
    # Load command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Run lap time simulation with optional parameter sweeps. \n Example usage: python sweep.py mass 250,350 --steps 10')
    parser.add_argument('param', type=str, help='Parameter name to sweep (e.g. "mass")')
    parser.add_argument('values', type=str, help='Comma-separated values for the parameter range (e.g. "250,350")')
    parser.add_argument('--steps', type=int, default=5, help='Number of steps in the parameter sweep (default: 5)')
    args = parser.parse_args()

    # Import modules
    from track.track import load_track
    from vehicle.vehicle import create_vehicle
    from simulator.simulator import run_lap_time_simulation

    print(f"Running parameter sweep for {args.param} with values {args.values} and {args.steps} steps")


    # Load track data
    print("Using track file_path:", config["track"]["file_path"])
    try:
        track = load_track(config["track"]["file_path"], config.get("debug_mode", False))
        print(f"Loaded track with {len(track.points)} points, total length: {track.total_length:.2f} m")
    except Exception as e:
        print("Error loading track:", e)
        return # unix exit code for bad input

    # Load vehicle parameters
    try:
        vehicle = create_vehicle(config)
        print(f"Loaded vehicle: {vehicle.params.name}, mass: {vehicle.params.mass} kg")
    except Exception as e:
        print("Error loading vehicle:", e)
        return 65 # unix exit code for bad input
    
    if not hasattr(vehicle.params, args.param):
        print(f"Error: Vehicle has no parameter named {args.param}")
        return 22 # unix exit code for invalid argument

    import numpy as np
    # Initialise simulation loop
    try:
        # run_lap_time_simulation(track, vehicle, config, display=False)
        param_values = [float(v) for v in args.values.split(',')]
        if len(param_values) == 1:
            param_values = [param_values[0]]  # Single value, no sweep
        else:
            param_values = np.linspace(param_values[0], param_values[1], args.steps)
        results = []
        for val in param_values:
            print(f"Running simulation with {args.param} = {val}")
            # Change the specified parameter in the vehicle
            if hasattr(vehicle.params, args.param):
                setattr(vehicle.params, args.param, val)
                print(f"Set vehicle parameter {args.param} to {val}")
            else:
                print(f"Warning: Vehicle has no parameter named {args.param}")
            sim_result = run_lap_time_simulation(track, vehicle, config, display=False)
            results.append((val, sim_result.lap_time))
        print("Parameter sweep results:")
        for val, lap_time in results:
            print(f"{args.param} = {val:.2f}, Lap Time = {lap_time:.2f} s")

    except Exception as e:
        print("Error during simulation:", e)
        return 0 # unix exit code for success

if __name__ == "__main__":
    exit(main())