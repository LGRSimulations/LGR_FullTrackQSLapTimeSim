import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

class LapTimeResults:
    """
    Flexible container for lap time simulation results.
    Supports dynamic addition of any result channel or array.
    """
    def __init__(self, **kwargs):
        # Store all provided keyword arguments as attributes
        for k, v in kwargs.items():
            setattr(self, k, v)

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        setattr(self, key, value)

    def keys(self):
        return self.__dict__.keys()

    def as_dict(self):
        return dict(self.__dict__)

def plot_lap_time_results(results):
    track = results.track
    finalSpeeds = results.finalSpeeds
    cornerSpeeds = results.cornerSpeeds
    lapTime = results.lapTime
    gLatChannel = results.gLatChannel
    gLongChannel = results.gLongChannel
    muLongChannel = results.muLongChannel
    muLatChannel = results.muLatChannel
    muCombinedChannel = results.muCombinedChannel
    signedMuLongChannel = results.signedMuLongChannel
    signedMuLatChannel = results.signedMuLatChannel
    normalLoadPerTyre = results.normalLoadPerTyre
    longForcePerTyre = results.longForcePerTyre
    latForcePerTyre = results.latForcePerTyre

    distances = [p.distance for p in track.points]
    xCoords = [p.x for p in track.points]
    yCoords = [p.y for p in track.points]
    vCar_kph = np.array(finalSpeeds) * 3.6  # Convert m/s to kph
    cornerSpeeds_kph = np.array(cornerSpeeds) * 3.6  # Convert m/s to kph

    # --- Plot 1: Speed Profile with gLat and gLong traces ---
    sns.set_theme(style="darkgrid", context="notebook")
    fig1, ax1 = plt.subplots(figsize=(12, 6))
    ax1.plot(distances, vCar_kph, label='vCar (kph)', linewidth=2, color='tab:blue')
    ax1.plot(distances, cornerSpeeds_kph, '--', label='Theoretical vCar limit', alpha=0.7, color='tab:gray')
    ax1.set_xlabel('Distance along track (m)')
    ax1.set_ylabel('Speed (kph)')
    ax1.set_ylim(0, 150)  # Limit left axis to 150 kph
    ax1.set_title(f'Lap Time: {lapTime:.2f} s\nLap Time Sim telemetry')
    ax1.legend(loc='upper left')
    # No grid for plot 1

    # Create second y-axis for gLat and gLong
    ax2 = ax1.twinx()
    ax2.plot(distances[1:], gLatChannel, label='gLat', color='tab:green')
    ax2.plot(distances[1:], gLongChannel, label='gLong', color='tab:orange')
    ax2.set_ylabel('Acceleration (g)')
    ax2.set_ylim(-10, 5)
    ax2.legend(loc='upper right')
    fig1.tight_layout()

    # --- Plot 2: G-G-V Diagram ---
    fig2, ax3 = plt.subplots(figsize=(8, 8))
    scatter_g = ax3.scatter(gLatChannel, gLongChannel, c=vCar_kph[1:], cmap='viridis', s=20)
    ax3.set_xlabel('Lateral Acceleration (g)')
    ax3.set_ylabel('Longitudinal Acceleration (g)')
    ax3.set_title('G-G-V Diagram')
    cbar_g = fig2.colorbar(scatter_g, ax=ax3)
    cbar_g.set_label('Speed (kph)')
    ax3.grid(True, which='both', linestyle='--', alpha=0.7)
    ax3.set_xlim(min(gLatChannel)-1, max(gLatChannel)+1)
    ax3.set_ylim(min(gLongChannel)-1, max(gLongChannel)+1)
    fig2.tight_layout()

    # --- Plot 3: 2D Track Map with vCar Colour Gradient ---
    fig3, ax4 = plt.subplots(figsize=(10, 8))
    scatter_track = ax4.scatter(xCoords, yCoords, c=vCar_kph, cmap='plasma', s=8)
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_title('2D Track Map: vCar (kph) Colour Gradient')
    cbar_track = fig3.colorbar(scatter_track, ax=ax4)
    cbar_track.set_label('Speed (kph)')
    ax4.axis('equal')
    fig3.tight_layout()

    # --- Plot 4: Mu Utilization Profile ---
    fig4, ax5 = plt.subplots(figsize=(12, 6))
    ax5.plot(distances[1:], muLongChannel, label='Longitudinal', linewidth=2, color='tab:red')
    ax5.plot(distances[1:], muLatChannel, label='Lateral', linewidth=2, color='tab:blue')
    ax5.plot(distances[1:], muCombinedChannel, label='Combined', linewidth=2, color='tab:purple', linestyle='--')
    ax5.set_xlabel('Distance along track (m)')
    ax5.set_ylabel('Friction Coefficient (μ)')
    ax5.set_title('Tyre Friction Coefficient Utilization')
    ax5.legend(loc='upper right')
    ax5.grid(True, alpha=0.3)
    ax5.set_ylim(0, max(muCombinedChannel) * 1.1)
    fig4.tight_layout()

    # --- Plot 5: Mu-Mu Diagram (Friction Ellipse) ---
    fig5, ax6 = plt.subplots(figsize=(8, 8))
    scatter_mu = ax6.scatter(signedMuLatChannel, signedMuLongChannel, c=vCar_kph[1:], cmap='plasma', s=20, alpha=0.6)
    ax6.set_xlabel('Lateral')
    ax6.set_ylabel('Longitudinal')
    ax6.set_title('Friction Ellipse (μ-Diagram)')
    
    # Draw friction circle reference
    theta = np.linspace(0, 2*np.pi, 100)
    if len(muCombinedChannel) > 0:
        mu_max = max(muCombinedChannel)
        circle_x = mu_max * np.cos(theta)
        circle_y = mu_max * np.sin(theta)
        ax6.plot(circle_x, circle_y, 'k--', alpha=0.3, label=f'abs(μ_max) = {mu_max:.2f}')
    
    cbar_mu = fig5.colorbar(scatter_mu, ax=ax6)
    cbar_mu.set_label('Speed (kph)')
    ax6.grid(True, which='both', linestyle='--', alpha=0.7)
    ax6.legend(loc='upper right')
    ax6.axis('equal')
    fig5.tight_layout()

    # --- Plot 6: 2D Track Map with Mu Combined Colour Gradient ---
    fig6, ax7 = plt.subplots(figsize=(10, 8))
    # Pad mu values to match track points length
    muCombined_padded = [0.0] + muCombinedChannel
    scatter_mu_track = ax7.scatter(xCoords, yCoords, c=muCombined_padded, cmap='hot', s=8)
    ax7.set_xlabel('X (m)')
    ax7.set_ylabel('Y (m)')
    ax7.set_title('2D Track Map: Combined Utilization')
    cbar_mu_track = fig6.colorbar(scatter_mu_track, ax=ax7)
    cbar_mu_track.set_label('Combined')
    ax7.axis('equal')
    fig6.tight_layout()

    # Show all plots at once
    plt.show()