
import numpy as np
import logging
from importlib import import_module

from .util.calcSpeedProfile import compute_speed_profile
from .util.results import LapTimeResults

logger = logging.getLogger(__name__)


def _plot_lap_time_results(results) -> None:
    try:
        plot_module = import_module(".util.prettyGraphs.lapTimeResults", __package__)
    except ModuleNotFoundError:
        logger.warning("Plotting extras are unavailable in this runtime; skipping lap-time plots.")
        return

    plot_module.plot_lap_time_results(results)

def run_lap_time_simulation(track, vehicle, config, display=True):
    """Initialize and run a lap time simulation for the whole track."""
    logger.info("Starting lap time simulation...")
    # Run the lap time sim, given the track and vehicle
    final_speeds, corner_speeds, diagnostics = compute_speed_profile(track, vehicle, config)

    # Compute lap time and g-forces based on final speeds
    lap_time = 0.0
    g_lat_channel = []
    g_long_channel = []
    normal_load_per_tyre = diagnostics.get('forward_normal_load_per_tyre', [])
    long_force_per_tyre = diagnostics.get('forward_net_long_force', [])
    lat_force_per_tyre = []
    for i in range(1, len(track.points)):
        ds = track.points[i].distance - track.points[i-1].distance
        v_prev = final_speeds[i-1]
        v_curr = final_speeds[i]
        v_avg = (v_curr + v_prev) / 2
        if ds > 0:
            g_long = ((v_curr**2 - v_prev**2) / (2 * ds)) / 9.81
        else:
            g_long = 0.0
        g_long_channel.append(g_long)
        g_lat = v_curr**2 * track.points[i].curvature / 9.81
        g_lat_channel.append(g_lat)
        if v_avg > 0:
            dt = ds / v_avg
            lap_time += dt
    logger.info(f"Estimated lap time: {lap_time:.2f} seconds")
    logger.info("Lap time simulation completed.")
    results = LapTimeResults(
        track=track,
        final_speeds=np.array(final_speeds),
        corner_speeds=np.array(corner_speeds),
        lap_time=lap_time,
        g_lat_channel=g_lat_channel,
        g_long_channel=g_long_channel,
        normal_load_per_tyre=normal_load_per_tyre,
        long_force_per_tyre=long_force_per_tyre,
        lat_force_per_tyre=lat_force_per_tyre,
        diagnostics=diagnostics,
    )
    if display:
        _plot_lap_time_results(results)
    return results
