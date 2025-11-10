import numpy as np
import logging
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.optimize import minimize

logger = logging.getLogger(__name__)

def computeVCarMax(gLat: float, curvature: float) -> float:
    """
    f2: Compute maximum theoretical speed for a given lateral acceleration and track curvature.
    
    v_max = sqrt(a_lat / curvature)
    
    Args:
        gLat: Lateral acceleration in m/s^2
        curvature: Track curvature in 1/m
        
    Returns:
        Maximum speed in m/s
    """
    
    # calculate vCarMax based on lateral acceleration and curvature
    vCarMax = np.sqrt(abs(gLat) / abs(curvature))   # m/s
    return vCarMax

def evaluateVehicleState(vCar: float, aSteer: float, aSideslip: float, curvature: float, vehicle, debugMode) -> dict:
    """
    f1: Evaluate vehicle state for given vCar, aSteer, aSideslip, and track curvature.
    
    Args:
        vCar: Vehicle speed (m/s)
        aSteer: Steering angle (degrees)
        aSideslip: Sideslip angle (degrees)
        curvature: Track curvature (1/m)
        vehicle: Vehicle object
        debugMode: Debug mode flag
        
    Returns:
        dict: {'gLat': float, 'M_z': float, 'is_valid': bool}
    """
    # Compute slip angles at front and rear (bicycle model)
    slipAngleFront = aSteer - aSideslip
    slipAngleRear = -aSideslip
    
    # Get tire forces
    F_front, F_rear = vehicle.computeTyreForces(slipAngleFront, slipAngleRear)
    
    # Compute yaw moment
    M_z = vehicle.computeYawMoment(F_front, F_rear, aSteer)
    
    # Compute lateral acceleration
    gLat = vehicle.computeLateralAcceleration(F_front, F_rear, vCar)

    # Check if state is valid (yaw moment close to zero)
    is_valid = abs(M_z) < 50.0  # Nm tolerance
    
    # debug info
    if debugMode:
        print(f"[DEBUG] variables: vCar={vCar:.2f}, gLat={gLat:.2f}, M_z={M_z:.2f}, is_valid={is_valid}, F_front={F_front:.2f}, F_rear={F_rear:.2f}")
    return {
        'gLat': gLat,
        'M_z': M_z,
        'is_valid': is_valid,
        'F_front': F_front,
        'F_rear': F_rear
    }

def findVehicleStateAtPoint(curvature: float, vehicle):
    """
    Finds the equilibrium vehicle state (speed, steering angle, sideslip) that satisfies yaw moment equilibrium
    and maximizes vehicle speed for a given track curvature using constrained optimization.

    The optimization seeks the highest possible speed (vCar) at which the vehicle can negotiate a curve of given curvature,
    subject to:
      - Yaw moment equilibrium (|M_z| < 50 Nm)
      - Lateral acceleration and curvature consistency
      - Maximum lateral acceleration (e.g., 2g limit)
      - Physical bounds on speed, steering, and sideslip

    For straight sections (curvature ≈ 0), returns a high speed with zero steering and sideslip.

    Args:
        curvature (float): Track curvature (1/m)
        vehicle: Vehicle object with parameters and tyre model

    Returns:
        dict: {
            'success': bool,         # Whether optimization succeeded
            'vCar': float,           # Maximum feasible speed at this curvature (m/s)
            'aSteer': float,         # Steering angle (degrees)
            'aSideslip': float       # Sideslip angle (degrees)
        }
    """
    # Handle straight sections (zero curvature)
    if abs(curvature) < 1e-3:
        # For straight sections, maximum speed is limited by drag/power
        # For now, return a high speed with zero steering
        return {
            'success': True,
            'vCar': 200.0,  # High speed for straights (adjust based on power limits)
            'aSteer': 0.0,
            'aSideslip': 0.0
        }
    

    # For debug: store first and last tried parameters and results
    debugMode = getattr(vehicle, 'config', {}).get('debugMode', False)
    debugInfo = {'first_x': None, 'first_result': None, 'last_x': None, 'last_result': None}

    def objective(x):
        vCar, aSteer, aSideslip = x
        result = evaluateVehicleState(vCar, aSteer, aSideslip, curvature, vehicle, debugMode)
        penalty = abs(result['M_z']) / 50.0 # Penalty for yaw moment deviation
        # Store first and last tried parameters/results if debugMode
        if debugMode:
            if debugInfo['first_x'] is None:
                debugInfo['first_x'] = x.copy() if hasattr(x, 'copy') else list(x)
                debugInfo['first_result'] = result.copy() if hasattr(result, 'copy') else dict(result)
            debugInfo['last_x'] = x.copy() if hasattr(x, 'copy') else list(x)
            debugInfo['last_result'] = result.copy() if hasattr(result, 'copy') else dict(result)
        return -vCar + penalty
    
    # Define constraint function for yaw moment equilibrium
    def constraintYawMoment(x):
        """
        Ensure that the yaw moment is close to zero. 
        We use a tolerance of 50 Nm.
        """
        vCar, aSteer, aSideslip = x
        result = evaluateVehicleState(vCar, aSteer, aSideslip, curvature, vehicle, debugMode)
        tolerance = 50.0  # Nm tolerance for yaw moment
        return tolerance - abs(result['M_z'])  # Must be >= 0
    
    # Define constraint function for lateral acceleration vs. curvature
    def constraintGLat(x):
        """ 
        Ensure that the lateral acceleration corresponds to a feasible maximum speed
        for the given curvature.
        """
        vCar, aSteer, aSideslip = x
        result = evaluateVehicleState(vCar, aSteer, aSideslip, curvature, vehicle, debugMode)
        vCarMax = computeVCarMax(result['gLat'], curvature)
        return vCarMax - vCar  # Must be >= 0
    
    def constraintMaxGLat(x):
        """
        Ensure that the lateral acceleration does not exceed a maximum limit, in this case 2G.
        """
        vCar, aSteer, aSideslip = x
        result = evaluateVehicleState(vCar, aSteer, aSideslip, curvature, vehicle, debugMode)
        return 4 - abs(result['gLat']) / 9.81     # for 2Gs

    # Initial guess based on simple bicycle model
    # Estimate initial steering angle from curvature and wheelbase
    L = vehicle.params.wheelbase
    initialASteer = np.degrees(abs(curvature) * L)      # Simple Ackermann (aSteer (radians) = curvature * L)
    initialASteer = np.clip(initialASteer, 0.1, 8.0)    # Reasonable range
    
    # Initial guess: conservative speed, estimated steering, small sideslip
    initialGuess = [5.0, initialASteer, 0.5]  # [vCar, aSteer, aSideslip]
    
    # Define bounds for optimization variables
    bounds = [
        (1.0, 150.0),     # vCar between 1 and 150 m/s
        (-30.0, 30.0),    # aSteer between -30 and 30 degrees
        (-5.0, 5.0)       # aSideslip between -5 and 5 degrees
    ]
    
    # Define constraints
    constraints = [
        {'type': 'ineq', 'fun': constraintYawMoment},
        {'type': 'ineq', 'fun': constraintGLat},
        {'type': 'ineq', 'fun': constraintMaxGLat}
    ]

    # Run optimization
    try:
        result = minimize(
            objective, 
            initialGuess,
            method='SLSQP',
            bounds=bounds,
            constraints=constraints,
            options={'disp': False, 'maxiter': 2000}
        )

        if result.success:
            vCar, aSteer, aSideslip = result.x
            logger.info(f"Optimization successful: vCar={vCar:.2f}, aSteer={aSteer:.2f}, aSideslip={aSideslip:.2f}")
            return {
                'success': True,
                'vCar': vCar,
                'aSteer': aSteer,
                'aSideslip': aSideslip
            }
        else:
            logger.warning(f"Optimization failed: {result.message}")
            if debugMode:
                print("[DEBUG] Optimization initial guess:", debugInfo['first_x'])
                print("[DEBUG] Initial evaluateVehicleState:", debugInfo['first_result'])
                print("[DEBUG] Optimization last tried x:", debugInfo['last_x'])
                print("[DEBUG] Last evaluateVehicleState:", debugInfo['last_result'])
            return {
                'success': False,
                'vCar': 0.0,
                'aSteer': 0.0,
                'aSideslip': 0.0
            }
    except Exception as e:
        logger.error(f"Optimization error: {str(e)}")
        if debugMode:
            print("[DEBUG] Exception during optimization:", str(e))
            print("[DEBUG] Optimization initial guess:", debugInfo['first_x'])
            print("[DEBUG] Initial evaluateVehicleState:", debugInfo['first_result'])
            print("[DEBUG] Optimization last tried x:", debugInfo['last_x'])
            print("[DEBUG] Last evaluateVehicleState:", debugInfo['last_result'])
        return {
            'success': False,
            'vCar': 0.0,
            'aSteer': 0.0,
            'aSideslip': 0.0
        }
    
def optimiseSpeedAtPoints(trackPoints, vehicle, config):
    """
    Find maximum vCar at multiple track points where equilibrium states exist.
    Uses direct constrained optimization for each point.
    Effectively gives us max cornering speeds given our vehicle model.
    
    Args:
        trackPoints: List of track points with curvature data
        vehicle: Vehicle object
        config: Config dict
        
    Returns:
        pointSpeeds: List of maximum speeds for each point (m/s)
    """
    pointSpeeds = []
    
    for point in trackPoints:
        curvature = point.curvature

        # Optimise vCar using constrained optimization
        result = findVehicleStateAtPoint(curvature, vehicle)
        baseMu = getattr(vehicle.tyreModel, 'baseMu')

        if result['success']:
            logger.info(f"Optimized vCar: {result['vCar']:.2f} m/s for curvature={curvature:.4f}")
            pointSpeeds.append(result['vCar'])
        else:
            logger.warning(f"Could not find equilibrium state for curvature={curvature:.4f}")
            logger.warning(f"This is point at coordinates x={point.x}, y={point.y}, z={point.z}")
            # Fallback: curvature-based max speed using baseMu
            g = 9.81  # m/s²
            if abs(curvature) > 1e-6:
                v_fallback = np.sqrt(baseMu * g / abs(curvature))
            else:
                v_fallback = 200.0  # For straight, use high speed
            logger.warning(f"Fallback: using v_fallback={v_fallback:.2f} m/s based on baseMu={baseMu}")
            pointSpeeds.append(v_fallback)
            
    return pointSpeeds

def forwardPass(track, vehicle, pointSpeeds):
    """
    Forward pass: acceleration-limited speed profile using powertrain model.
    Propagates speeds forward using available engine forces minus drag.
    
    At each segment:
    - Select optimal gear for current speed
    - Compute available traction force from powertrain (F_x = T_wheel / r_wheel)
    - Subtract aerodynamic drag
    - Update speed via kinematics: v_pred = sqrt(v_prev^2 + 2 * a_lon * ds)
    
    Uses all track points.
    """
    nPoints = len(track.points)
    speeds = np.zeros(nPoints)
    speeds[0] = pointSpeeds[0]  # start at first point speed

    for i in range(1, nPoints):
        # Distance to next point
        ds = track.points[i].distance - track.points[i-1].distance
        
        if ds <= 0:
            speeds[i] = speeds[i-1]
            continue
        
        # Previous speed - use actual speed from previous point
        vPrev = speeds[i-1]
        
        # Use minimum speed threshold for calculations to avoid numerical issues
        vCalc = max(vPrev, 1.0)
        
        # Select optimal gear for this speed
        gearRatio = vehicle.selectOptimalGear(vCalc)
        
        # Compute net longitudinal force (traction - drag) at previous speed
        F_x = vehicle.computeLongitudinalForce(vCalc, gearRatio, throttle=1.0)
        
        # Compute longitudinal acceleration
        a_lon = F_x / vehicle.params.mass
        
        # Compute predicted speed using kinematics: v^2 = v0^2 + 2*a*ds
        v_pred_squared = vPrev**2 + 2 * a_lon * ds
        
        if v_pred_squared < 0:
            v_pred = vPrev
        else:
            v_pred = np.sqrt(v_pred_squared)
        
        # Take minimum of acceleration-limited and corner-limited speed
        v_pred = min(v_pred, pointSpeeds[i])
        speeds[i] = v_pred

    return speeds

def backwardPass(track, vehicle, pointSpeeds):
    """
    Backward pass: braking-limited speed profile using tyre model.
    
    Propagates speeds backward from end to start. At each segment:
    - Compute maximum braking force from tyre longitudinal grip (using DX_LUT)
    - Calculate maximum deceleration: a_brake = F_brake / mass
    - Work backwards: v_i = sqrt(v_{i+1}^2 + 2 * a_brake * ds)
    - Enforce v_i <= pointSpeeds[i] (corner limit)
    
    Uses all track points.
    """
    nPoints = len(track.points)
    speeds = np.zeros(nPoints)
    speeds[-1] = pointSpeeds[-1]  # end at last point speed

    for i in range(nPoints-2, -1, -1):
        # Distance to next point
        ds = track.points[i+1].distance - track.points[i].distance
        
        if ds <= 0:
            speeds[i] = speeds[i+1]
            continue
        
        # Next speed (where we need to brake to)
        vNext = speeds[i+1]
        
        # Compute normal load per tire (static, no load transfer for now)
        normalLoadPerTire = vehicle.computeStaticNormalLoad()
        
        # Get longitudinal grip multiplier from tyre model (DX)
        # The tyre model will return the peak longitudinal force for this load
        # For braking, we assume maximum slip ratio to get peak force
        peak_slip_ratio = 12.0  # percent, typical peak for most tyres
        
        # Get longitudinal force per tire at peak slip (braking)
        # Multiply by 4 for all tires (assuming all-wheel braking)
        F_brake_per_tire = vehicle.tyreModel.getLongitudinalForce(
            slipRatio=-peak_slip_ratio,  # Negative for braking
            normalLoad=normalLoadPerTire
        )
        
        # Total braking force (all 4 tires)
        F_brake_total = abs(F_brake_per_tire) * 4
        
        # Add aerodynamic drag assistance (drag helps when braking)
        # Use average speed for drag calculation
        vAvg = vNext  # Conservative estimate
        F_drag = vehicle.computeAeroDrag(vAvg)
        
        # Total braking capability
        F_brake_total_with_drag = F_brake_total + F_drag
        
        # Maximum deceleration
        aBrake = F_brake_total_with_drag / vehicle.params.mass
        
        # Maximum speed we can be at and still brake to vNext
        # v_current^2 = v_next^2 + 2*a*ds
        vBrake_squared = vNext**2 + 2 * aBrake * ds
        
        if vBrake_squared < 0:
            vBrake = vNext
        else:
            vBrake = np.sqrt(vBrake_squared)
        
        # Take minimum of braking-limited and corner-limited speed
        speeds[i] = min(vBrake, pointSpeeds[i])

    return speeds

def computeSpeedProfile(track, vehicle, config):
    """
    Compute final speed profile using forward and backward passes for all track points.
    Parallelises the optimisation of cornering speeds at each point.
    """
    # Step 1: For each track point, optimise max cornering speed given vehicle model
    pointSpeeds = optimiseSpeedAtPoints(track.points, vehicle, config)
    
    # Step 2: Forward Pass (acceleration limited)
    forwardSpeeds = forwardPass(track, vehicle, pointSpeeds)

    # Step 3: Backward Pass (braking limited)
    backwardSpeeds = backwardPass(track, vehicle, pointSpeeds)

    # Step 4: Take min of forward and backward pass speeds
    finalSpeeds = np.minimum(forwardSpeeds, backwardSpeeds)
    return finalSpeeds, pointSpeeds

def runLapTimeSimulation(track, vehicle, config) -> None:
    """Initialize and run a lap time simulation for the whole track."""
    logger.info("Starting lap time simulation...")
    
    # Compute speed profile
    finalSpeeds, cornerSpeeds = computeSpeedProfile(track, vehicle, config)

    # Compute lap time and collect acceleration traces and mu values
    lapTime = 0.0
    gLatChannel = []
    gLongChannel = []
    muLongChannel = []  # Longitudinal mu (accel/brake)
    muLatChannel = []   # Lateral mu (cornering)
    muCombinedChannel = []  # Combined mu utilization
    signedMuLongChannel = []  # Signed longitudinal mu
    signedMuLatChannel = []   # Signed lateral mu
    normalLoadPerTyre = []
    longForcePerTyre = []
    latForcePerTyre = []
    
    for i in range(1, len(track.points)):
        ds = track.points[i].distance - track.points[i-1].distance
        vPrev = finalSpeeds[i-1]
        vCurr = finalSpeeds[i]
        vAvg = (vCurr + vPrev) / 2
        
        # Longitudinal acceleration (finite difference)
        if ds > 0:
            # a = (v^2 - u^2) / (2*s)
            gLong = ((vCurr**2 - vPrev**2) / (2 * ds)) / 9.81
        else:
            gLong = 0.0
        gLongChannel.append(gLong)
        
        # Lateral acceleration (from curvature)
        # a_lat = v^2 * curvature
        gLat = vCurr**2 * track.points[i].curvature / 9.81
        gLatChannel.append(gLat)
        
        # Compute normal load per tyre (static, could add load transfer later)
        Fz_per_tyre = vehicle.computeStaticNormalLoad()
        normalLoadPerTyre.append(Fz_per_tyre)
        
        # Compute longitudinal force per tyre
        # F_long = m * a_long
        F_long_total = gLong * 9.81 * vehicle.params.mass  # Total longitudinal force
        F_long_per_tyre = F_long_total / 4.0  # Assume equal distribution to 4 tyres
        longForcePerTyre.append(F_long_per_tyre)
        
        # Compute lateral force per tyre
        # F_lat = m * a_lat
        F_lat_total = gLat * 9.81 * vehicle.params.mass  # Total lateral force
        F_lat_per_tyre = F_lat_total / 4.0  # Assume equal distribution to 4 tyres
        latForcePerTyre.append(F_lat_per_tyre)
        
        # Calculate mu values
        if Fz_per_tyre > 0:
            mu_long = abs(F_long_per_tyre) / Fz_per_tyre        # Formula: mu = F / Fz
            signedMuLong = np.sign(F_long_per_tyre) * mu_long   # Formula: signedMu = sign(F) * mu
            mu_lat = abs(F_lat_per_tyre) / Fz_per_tyre          # Formula: mu = F / Fz
            signedMuLat = np.sign(F_lat_per_tyre) * mu_lat      # Formula: signedMu = sign(F) * mu
            # Combined mu using friction circle
            mu_combined = np.sqrt(mu_long**2 + mu_lat**2)
        else:
            mu_long = 0.0
            mu_lat = 0.0
            mu_combined = 0.0
        
        muLongChannel.append(mu_long)
        muLatChannel.append(mu_lat)
        signedMuLongChannel.append(signedMuLong)
        signedMuLatChannel.append(signedMuLat)
        muCombinedChannel.append(mu_combined)
        
        if vAvg > 0:
            dt = ds / vAvg
            lapTime += dt
    
    logger.info(f"Estimated lap time: {lapTime:.2f} seconds")
    logger.info("Lap time simulation completed.")
    
    # from src.simulator.simulatorLogger import exportTelemetryData
    # Export telemetry data for FEA analysis
    # exportTelemetryData(track, finalSpeeds, gLatChannel, gLongChannel, 
    #                    muLongChannel, muLatChannel, muCombinedChannel,
    #                    normalLoadPerTyre, longForcePerTyre, latForcePerTyre,
    #                    config)

    # Prepare arrays for plotting
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