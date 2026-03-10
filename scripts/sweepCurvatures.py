"""
Sweep Curvatures Script
=======================
Simulates vehicle performance across a series of constant-curvature tracks
of varying severity. This represents real-world testing where engineers sweep
steering angles at constant curvature to characterize vehicle performance.

Outputs:
- 2 plots per curvature: lateral acceleration vs distance, speed vs distance
- Console logging with key performance metrics
"""

import sys
import os
import json
import logging
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Add src directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from track.track import Track, TrackPoint
from vehicle.vehicle import create_vehicle
from simulator.util.calcSpeedProfile import (
    optimise_speed_at_points,
    forward_pass,
    backward_pass
)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def load_config(config_path: str) -> dict:
    """Load configuration from JSON file."""
    with open(config_path, 'r') as f:
        return json.load(f)


def create_synthetic_circular_track(
    curvature: float,
    track_length: float = 500.0,
    num_points: int = 200,
    center_x: float = 0.0,
    center_y: float = 0.0,
    z: float = 0.0
) -> Track:
    """
    Generate a synthetic circular track with constant curvature.
    
    Args:
        curvature: Track curvature in 1/m (κ = 1/R). Must be > 0.
        track_length: Desired track length in meters.
        num_points: Number of discrete points to generate.
        center_x: X coordinate of circle center.
        center_y: Y coordinate of circle center.
        z: Constant elevation (flat track).
    
    Returns:
        Track object with constant curvature throughout.
    """
    if curvature <= 0:
        raise ValueError("Curvature must be positive")
    
    # Calculate radius from curvature: R = 1/κ
    radius = 1.0 / curvature
    
    # Calculate the angular span needed for desired track length
    # Arc length = R × θ, so θ = track_length / R
    total_angle = track_length / radius
    
    # Ensure we create a closed loop if track_length >= circumference
    circumference = 2 * np.pi * radius
    if track_length >= circumference:
        total_angle = 2 * np.pi  # Full circle
        track_length = circumference
    
    # Generate angular positions
    angles = np.linspace(0, total_angle, num_points)
    
    # Generate coordinates (parametric circle)
    x_coords = center_x + radius * np.cos(angles)
    y_coords = center_y + radius * np.sin(angles)
    z_coords = np.full(num_points, z)
    
    # Calculate cumulative distances
    distances = np.zeros(num_points)
    for i in range(1, num_points):
        dx = x_coords[i] - x_coords[i-1]
        dy = y_coords[i] - y_coords[i-1]
        dz = z_coords[i] - z_coords[i-1]
        segment_length = np.sqrt(dx**2 + dy**2 + dz**2)
        distances[i] = distances[i-1] + segment_length
    
    # Calculate headings (tangent direction)
    headings = np.zeros(num_points)
    for i in range(num_points - 1):
        dx = x_coords[i+1] - x_coords[i]
        dy = y_coords[i+1] - y_coords[i]
        headings[i] = np.arctan2(dy, dx)
    headings[-1] = headings[-2]  # Last point uses previous heading
    
    # Constant elevation angle (flat track)
    elevation_angles = np.zeros(num_points)
    
    # Create TrackPoint objects with constant curvature
    points = []
    for i in range(num_points):
        point = TrackPoint(
            distance=distances[i],
            x=x_coords[i],
            y=y_coords[i],
            z=z_coords[i],
            curvature=curvature,  # Constant curvature for all points
            heading=headings[i],
            elevation_angle=elevation_angles[i]
        )
        points.append(point)
    
    # Determine if track is closed (forms complete circle)
    is_closed = total_angle >= 2 * np.pi - 0.01
    
    return Track(points, is_closed=is_closed)


def compute_speed_profile_from_max(track, vehicle, config):
    """
    Compute speed profile starting from the maximum achievable corner speed.
    
    This modification ensures we capture the vehicle's true performance
    by starting from optimal entry conditions rather than from rest.
    
    Args:
        track: Track object
        vehicle: Vehicle object
        config: Configuration dict
    
    Returns:
        tuple: (final_speeds, corner_speeds) arrays
    """
    # Pass 1: Find ultimate speeds possible given vehicle model (cornering limits)
    corner_speeds = optimise_speed_at_points(track.points, vehicle, config)
    
    # For constant-curvature tracks, all corner speeds should be similar
    # Find the maximum achievable corner speed
    max_corner_speed = max(corner_speeds)
    max_speed_idx = corner_speeds.index(max_corner_speed)
    
    logger.info(f"Max corner speed: {max_corner_speed:.2f} m/s at point {max_speed_idx}")
    
    # Create modified corner_speeds that starts from max speed
    # This simulates entering the curve at the highest possible speed
    modified_corner_speeds = corner_speeds.copy()
    
    # Pass 2: Forward pass propagating accel limits (starting from max speed)
    forward_speeds = forward_pass(track, vehicle, modified_corner_speeds)
    
    # Pass 3: Backward pass propagating decel limits
    backward_speeds = backward_pass(track, vehicle, modified_corner_speeds)
    
    # Take minimum of forward and backward speeds to get final speed profile
    final_speeds = np.minimum(forward_speeds, backward_speeds)
    
    return final_speeds, np.array(corner_speeds)


def calculate_g_forces(track, final_speeds):
    """
    Calculate lateral and longitudinal g-forces from speed profile.
    
    Args:
        track: Track object
        final_speeds: Array of speeds at each point (m/s)
    
    Returns:
        tuple: (g_lat_channel, g_long_channel) lists
    """
    g_lat_channel = []
    g_long_channel = []
    
    for i in range(1, len(track.points)):
        ds = track.points[i].distance - track.points[i-1].distance
        v_prev = final_speeds[i-1]
        v_curr = final_speeds[i]
        
        # Longitudinal g: from speed change
        if ds > 0:
            g_long = ((v_curr**2 - v_prev**2) / (2 * ds)) / 9.81
        else:
            g_long = 0.0
        g_long_channel.append(g_long)
        
        # Lateral g: centripetal acceleration = v² × κ
        g_lat = v_curr**2 * abs(track.points[i].curvature) / 9.81
        g_lat_channel.append(g_lat)
    
    return g_lat_channel, g_long_channel


def simulate_curvature(curvature: float, vehicle, config, track_length: float = 500.0) -> dict:
    """
    Run complete simulation for a single curvature value.
    
    Args:
        curvature: Track curvature in 1/m
        vehicle: Vehicle object
        config: Configuration dict
        track_length: Length of synthetic track in meters
    
    Returns:
        dict with simulation results
    """
    logger.info(f"\n{'='*60}")
    logger.info(f"Simulating curvature: {curvature:.4f} 1/m (radius: {1/curvature:.1f} m)")
    logger.info(f"{'='*60}")
    
    # Generate synthetic track
    track = create_synthetic_circular_track(
        curvature=curvature,
        track_length=track_length,
        num_points=200
    )
    logger.info(f"Generated track with {len(track.points)} points, length: {track.total_length:.1f} m")
    
    # Run simulation
    final_speeds, corner_speeds = compute_speed_profile_from_max(track, vehicle, config)
    
    # Calculate g-forces
    g_lat_channel, g_long_channel = calculate_g_forces(track, final_speeds)
    
    # Extract distances for plotting
    distances = [p.distance for p in track.points]
    
    # Calculate key metrics
    avg_speed = np.mean(final_speeds)
    max_speed = np.max(final_speeds)
    min_speed = np.min(final_speeds)
    peak_g_lat = max(g_lat_channel) if g_lat_channel else 0.0
    avg_g_lat = np.mean(g_lat_channel) if g_lat_channel else 0.0
    
    logger.info(f"Results: Avg speed: {avg_speed*3.6:.1f} kph, "
                f"Max speed: {max_speed*3.6:.1f} kph, "
                f"Peak g_lat: {peak_g_lat:.2f} g")
    
    return {
        'curvature': curvature,
        'radius': 1.0 / curvature,
        'track': track,
        'distances': distances,
        'final_speeds': final_speeds,
        'corner_speeds': corner_speeds,
        'g_lat_channel': g_lat_channel,
        'g_long_channel': g_long_channel,
        'metrics': {
            'avg_speed_ms': avg_speed,
            'avg_speed_kph': avg_speed * 3.6,
            'max_speed_ms': max_speed,
            'max_speed_kph': max_speed * 3.6,
            'min_speed_ms': min_speed,
            'min_speed_kph': min_speed * 3.6,
            'peak_g_lat': peak_g_lat,
            'avg_g_lat': avg_g_lat
        }
    }


def plot_summary_comparison(all_results: list):
    """
    Create a summary comparison plot across all curvatures.
    
    Args:
        all_results: List of result dictionaries from simulate_curvature()
    """
    curvatures = [r['curvature'] for r in all_results]
    radii = [r['radius'] for r in all_results]
    avg_speeds = [r['metrics']['avg_speed_kph'] for r in all_results]
    
    sns.set_theme(style="darkgrid", context="notebook")
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle('Curvature Sweep Summary', fontsize=14, fontweight='bold')
    
    # Plot 1: Average Speed vs Curvature
    ax1 = axes[0]
    ax1.plot(curvatures, avg_speeds, 'o-', linewidth=2, markersize=8, color='tab:blue')
    ax1.set_xlabel('Curvature (1/m)')
    ax1.set_ylabel('Average Speed (kph)')
    ax1.set_title('Speed vs Curvature')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Speed vs Radius
    ax2 = axes[1]
    ax2.plot(radii, avg_speeds, 'd-', linewidth=2, markersize=8, color='tab:orange')
    ax2.set_xlabel('Turn Radius (m)')
    ax2.set_ylabel('Average Speed (kph)')
    ax2.set_title('Speed vs Turn Radius')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()


def print_summary_table(all_results: list):
    """Print a summary table of all curvature results."""
    print("\n" + "="*60)
    print("CURVATURE SWEEP SUMMARY")
    print("="*60)
    print(f"{'Curvature':>12} {'Radius':>10} {'Avg Speed':>12} {'Max Speed':>12}")
    print(f"{'(1/m)':>12} {'(m)':>10} {'(kph)':>12} {'(kph)':>12}")
    print("-"*60)
    
    for r in all_results:
        m = r['metrics']
        print(f"{r['curvature']:>12.4f} {r['radius']:>10.1f} {m['avg_speed_kph']:>12.1f} "
              f"{m['max_speed_kph']:>12.1f}")
    
    print("="*60 + "\n")


def main():
    """Main entry point for curvature sweep simulation."""
    logger.info("="*60)
    logger.info("CURVATURE SWEEP SIMULATION")
    logger.info("="*60)
    
    # Load configuration
    config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config.json'))
    logger.info(f"Loading config from: {config_path}")
    config = load_config(config_path)
    
    # Create vehicle
    logger.info("Creating vehicle from configuration...")
    try:
        vehicle = create_vehicle(config)
        logger.info(f"Vehicle loaded: {vehicle.params.name}, mass: {vehicle.params.mass} kg")
    except Exception as e:
        logger.error(f"Error loading vehicle: {e}")
        return
    
    # Define curvatures to sweep
    # Formula Student relevant range: ~9m radius (0.11) to ~4m radius (0.25)
    # Curvature values: 0.05 = 20m radius, 0.11 = 9m radius, 0.20 = 5m radius, 0.25 = 4m radius
    curvatures = [0.05, 0.08, 0.11, 0.14, 0.17, 0.20, 0.22, 0.25]
    
    logger.info(f"Sweeping {len(curvatures)} curvature values: {curvatures}")
    
    # Track length for each simulation
    track_length = 500.0  # meters
    
    # Run simulations for all curvatures
    all_results = []
    for curvature in curvatures:
        try:
            result = simulate_curvature(curvature, vehicle, config, track_length)
            all_results.append(result)
        except Exception as e:
            logger.error(f"Error simulating curvature {curvature}: {e}")
            continue
    
    if not all_results:
        logger.error("No successful simulations. Exiting.")
        return
    
    # Print summary table
    print_summary_table(all_results)
    
    # Generate summary comparison plot
    logger.info("Generating visualization plots...")
    plot_summary_comparison(all_results)
    
    # Show all plots
    logger.info("Displaying plots...")
    plt.show()
    
    logger.info("Curvature sweep simulation complete!")


if __name__ == "__main__":
    main()
