
import numpy as np
import logging
import pandas as pd
import os
from datetime import datetime
logger = logging.getLogger(__name__)

def exportTelemetryData(track, finalSpeeds, gLatChannel, gLongChannel,
                       muLongChannel, muLatChannel, muCombinedChannel,
                       normalLoadPerTyre, longForcePerTyre, latForcePerTyre,
                       config):
    """
    Export detailed telemetry data including mu values for FEA analysis.
    
    Args:
        track: Track object with points
        finalSpeeds: Array of speeds at each point
        gLatChannel: Lateral acceleration in g's
        gLongChannel: Longitudinal acceleration in g's
        muLongChannel: Longitudinal mu values
        muLatChannel: Lateral mu values
        muCombinedChannel: Combined mu values
        normalLoadPerTyre: Normal load per tyre in N
        longForcePerTyre: Longitudinal force per tyre in N
        latForcePerTyre: Lateral force per tyre in N
        config: Configuration dictionary
    """
    # Create output directory if it doesn't exist
    output_dir = config.get('outputDirectory', 'output')
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate timestamp for filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"telemetry_data_{timestamp}.csv"
    filepath = os.path.join(output_dir, filename)
    
    # Prepare data for export
    # Note: channels are 1 element shorter than track points (differential values)
    nPoints = len(track.points)
    
    # Create arrays with proper lengths
    distances = [track.points[i].distance for i in range(nPoints)]
    xCoords = [track.points[i].x for i in range(nPoints)]
    yCoords = [track.points[i].y for i in range(nPoints)]
    zCoords = [track.points[i].z for i in range(nPoints)]
    curvatures = [track.points[i].curvature for i in range(nPoints)]
    speeds_kph = finalSpeeds * 3.6  # Convert m/s to kph
    
    # Pad differential channels with initial value (0) to match length
    gLat_padded = [0.0] + gLatChannel
    gLong_padded = [0.0] + gLongChannel
    muLong_padded = [0.0] + muLongChannel
    muLat_padded = [0.0] + muLatChannel
    muCombined_padded = [0.0] + muCombinedChannel
    normalLoad_padded = [normalLoadPerTyre[0] if normalLoadPerTyre else 0.0] + normalLoadPerTyre
    longForce_padded = [0.0] + longForcePerTyre
    latForce_padded = [0.0] + latForcePerTyre
    
    # Classify driving phase (acceleration, braking, cornering, combined)
    drivingPhase = []
    for i in range(len(gLong_padded)):
        gLong = gLong_padded[i]
        gLat = abs(gLat_padded[i])
        
        if abs(gLong) > 0.1 and gLat > 0.3:
            phase = "Combined"
        elif gLong > 0.1:
            phase = "Acceleration"
        elif gLong < -0.1:
            phase = "Braking"
        elif gLat > 0.3:
            phase = "Cornering"
        else:
            phase = "Cruise"
        drivingPhase.append(phase)
    
    # Create DataFrame
    df = pd.DataFrame({
        'Distance_m': distances,
        'X_m': xCoords,
        'Y_m': yCoords,
        'Z_m': zCoords,
        'Curvature_1/m': curvatures,
        'Speed_kph': speeds_kph,
        'gLat_g': gLat_padded,
        'gLong_g': gLong_padded,
        'mu_Longitudinal': muLong_padded,
        'mu_Lateral': muLat_padded,
        'mu_Combined': muCombined_padded,
        'NormalLoad_per_Tyre_N': normalLoad_padded,
        'LongForce_per_Tyre_N': longForce_padded,
        'LatForce_per_Tyre_N': latForce_padded,
        'DrivingPhase': drivingPhase
    })
    
    # Export to CSV
    df.to_csv(filepath, index=False)
    logger.info(f"Telemetry data exported to: {filepath}")
    
    # Generate summary statistics for FEA
    summary_filepath = os.path.join(output_dir, f"mu_summary_{timestamp}.txt")
    with open(summary_filepath, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("MU VALUES SUMMARY FOR FEA ANALYSIS\n")
        f.write("=" * 80 + "\n\n")
        
        # Overall statistics
        f.write("OVERALL STATISTICS:\n")
        f.write("-" * 80 + "\n")
        f.write(f"Max mu (combined): {max(muCombined_padded):.3f}\n")
        f.write(f"Max mu (longitudinal): {max(muLong_padded):.3f}\n")
        f.write(f"Max mu (lateral): {max(muLat_padded):.3f}\n")
        f.write(f"Max gLat: {max([abs(g) for g in gLat_padded]):.2f} g\n")
        f.write(f"Max gLong (accel): {max(gLong_padded):.2f} g\n")
        f.write(f"Max gLong (brake): {min(gLong_padded):.2f} g\n\n")
        
        # Phase-specific statistics
        phases = ["Acceleration", "Braking", "Cornering", "Combined"]
        for phase in phases:
            phase_data = df[df['DrivingPhase'] == phase]
            if len(phase_data) > 0:
                f.write(f"\n{phase.upper()} PHASE:\n")
                f.write("-" * 80 + "\n")
                f.write(f"Occurrences: {len(phase_data)} points\n")
                f.write(f"mu_Long - Mean: {phase_data['mu_Longitudinal'].mean():.3f}, ")
                f.write(f"Max: {phase_data['mu_Longitudinal'].max():.3f}, ")
                f.write(f"95th percentile: {phase_data['mu_Longitudinal'].quantile(0.95):.3f}\n")
                f.write(f"mu_Lat - Mean: {phase_data['mu_Lateral'].mean():.3f}, ")
                f.write(f"Max: {phase_data['mu_Lateral'].max():.3f}, ")
                f.write(f"95th percentile: {phase_data['mu_Lateral'].quantile(0.95):.3f}\n")
                f.write(f"mu_Combined - Mean: {phase_data['mu_Combined'].mean():.3f}, ")
                f.write(f"Max: {phase_data['mu_Combined'].max():.3f}, ")
                f.write(f"95th percentile: {phase_data['mu_Combined'].quantile(0.95):.3f}\n")
                f.write(f"Normal Load per Tyre - Mean: {phase_data['NormalLoad_per_Tyre_N'].mean():.1f} N, ")
                f.write(f"Max: {phase_data['NormalLoad_per_Tyre_N'].max():.1f} N\n")
        
        f.write("\n" + "=" * 80 + "\n")
        f.write("RECOMMENDED FEA LOAD CASES:\n")
        f.write("=" * 80 + "\n\n")
        
        # Peak braking case
        max_brake_idx = gLong_padded.index(min(gLong_padded))
        f.write("1. PEAK BRAKING CASE:\n")
        f.write(f"   Distance: {distances[max_brake_idx]:.2f} m\n")
        f.write(f"   gLong: {gLong_padded[max_brake_idx]:.2f} g\n")
        f.write(f"   mu_Long: {muLong_padded[max_brake_idx]:.3f}\n")
        f.write(f"   Normal Load per Tyre: {normalLoad_padded[max_brake_idx]:.1f} N\n")
        f.write(f"   Long Force per Tyre: {longForce_padded[max_brake_idx]:.1f} N\n\n")
        
        # Peak acceleration case
        max_accel_idx = gLong_padded.index(max(gLong_padded))
        f.write("2. PEAK ACCELERATION CASE:\n")
        f.write(f"   Distance: {distances[max_accel_idx]:.2f} m\n")
        f.write(f"   gLong: {gLong_padded[max_accel_idx]:.2f} g\n")
        f.write(f"   mu_Long: {muLong_padded[max_accel_idx]:.3f}\n")
        f.write(f"   Normal Load per Tyre: {normalLoad_padded[max_accel_idx]:.1f} N\n")
        f.write(f"   Long Force per Tyre: {longForce_padded[max_accel_idx]:.1f} N\n\n")
        
        # Peak cornering case
        max_lat_idx = [abs(g) for g in gLat_padded].index(max([abs(g) for g in gLat_padded]))
        f.write("3. PEAK CORNERING CASE:\n")
        f.write(f"   Distance: {distances[max_lat_idx]:.2f} m\n")
        f.write(f"   gLat: {gLat_padded[max_lat_idx]:.2f} g\n")
        f.write(f"   mu_Lat: {muLat_padded[max_lat_idx]:.3f}\n")
        f.write(f"   Normal Load per Tyre: {normalLoad_padded[max_lat_idx]:.1f} N\n")
        f.write(f"   Lat Force per Tyre: {latForce_padded[max_lat_idx]:.1f} N\n\n")
        
        # Peak combined case
        max_combined_idx = muCombined_padded.index(max(muCombined_padded))
        f.write("4. PEAK COMBINED CASE:\n")
        f.write(f"   Distance: {distances[max_combined_idx]:.2f} m\n")
        f.write(f"   gLong: {gLong_padded[max_combined_idx]:.2f} g\n")
        f.write(f"   gLat: {gLat_padded[max_combined_idx]:.2f} g\n")
        f.write(f"   mu_Combined: {muCombined_padded[max_combined_idx]:.3f}\n")
        f.write(f"   Normal Load per Tyre: {normalLoad_padded[max_combined_idx]:.1f} N\n")
        f.write(f"   Long Force per Tyre: {longForce_padded[max_combined_idx]:.1f} N\n")
        f.write(f"   Lat Force per Tyre: {latForce_padded[max_combined_idx]:.1f} N\n\n")
    
    logger.info(f"Mu summary exported to: {summary_filepath}")
    print(f"\nTelemetry data exported to: {filepath}")
    print(f"Mu summary exported to: {summary_filepath}")
