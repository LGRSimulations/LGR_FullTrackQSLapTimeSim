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

def evaluateVehicleState(vCar: float, aSteer: float, aSideslip: float, curvature: float, vehicle):
    """
    f1: Evaluate vehicle state for given vCar, aSteer, aSideslip, and track curvature.
    
    Args:
        vCar: Vehicle speed (m/s)
        aSteer: Steering angle (degrees)
        aSideslip: Sideslip angle (degrees)
        curvature: Track curvature (1/m)
        vehicle: Vehicle object
        
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
    
    # Check if state is valid (yaw moment near zero)
    is_valid = abs(M_z) < 1e-1
    
    return {
        'gLat': gLat,
        'M_z': M_z,
        'is_valid': is_valid,
        'F_front': F_front,
        'F_rear': F_rear
    }

def findVehicleStateAtPoint(curvature: float, vehicle):
    """
    Find vehicle state (vCar, aSteer, aSideslip) that satisfies yaw moment = 0
    and maximizes vCar for given track curvature using constrained optimization.

    We minimise objective function f(x) = vCar
    Function 1: We optimise using a comprehensive bicycle model, what's the potential gLat & yaw moment we can get
                We are NOT constraining yaw moment here, we are finding our theoretical max gLat for given vCar, aSteer, aSideslip
                Where f1 is (glat, yaw moment) = f1(vCar, aSteer, aSideslip, curvature)
    
    Function 2: We compute theoretical max vCar for given gLat & curvature.
                Here we constrain yaw moment = 0, and we are looking for the maximum vCar we can achieve
                Where f2 is vCarMax = f2(gLat, curvature)

    
    Args:
        curvature: Track curvature (1/m)
        vehicle: Vehicle object
    Returns:
        dict: {'success': bool, 'vCar': float, 'aSteer': float, 'aSideslip': float}
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
    
    # Define objective function to maximize vCar (by minimizing negative vCar)
    def objective(x):
        vCar, aSteer, aSideslip = x
        return -vCar  # We minimize negative vCar to maximize vCar
    
    # Define constraint function for yaw moment equilibrium
    def constraintYawMoment(x):
        vCar, aSteer, aSideslip = x
        result = evaluateVehicleState(vCar, aSteer, aSideslip, curvature, vehicle)
        tolerance = 50.0  # Nm tolerance for yaw moment
        return tolerance - abs(result['M_z'])  # Must be >= 0
    
    # Define constraint function for lateral acceleration vs. curvature
    def constraintGLat(x):
        vCar, aSteer, aSideslip = x
        result = evaluateVehicleState(vCar, aSteer, aSideslip, curvature, vehicle)
        vCarMax = computeVCarMax(result['gLat'], curvature)
        return vCarMax - vCar  # Must be >= 0
    
    # Initial guess based on simple bicycle model
    # Estimate initial steering angle from curvature and wheelbase
    L = vehicle.params.wheelbase
    initialASteer = np.degrees(abs(curvature) * L)  # Simple Ackermann
    initialASteer = np.clip(initialASteer, 0.1, 8.0)  # Reasonable range
    
    # Initial guess: conservative speed, estimated steering, small sideslip
    initialGuess = [15.0, initialASteer, 0.5]  # [vCar, aSteer, aSideslip]
    
    # Define bounds for optimization variables
    bounds = [
        (1.0, 150.0),     # vCar between 1 and 150 m/s
        (-30.0, 30.0),    # aSteer between -30 and 30 degrees
        (-5.0, 5.0)       # aSideslip between -5 and 5 degrees
    ]
    
    # Define constraints
    constraints = [
        {'type': 'ineq', 'fun': constraintYawMoment},
        {'type': 'ineq', 'fun': constraintGLat}
    ]
    
    # Run optimization
    try:
        result = minimize(
            objective, 
            initialGuess,
            method='SLSQP',     # We use the SLSQP method for constrained optimization
            bounds=bounds,
            constraints=constraints,
            options={'disp': False, 'maxiter': 100}
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
            return {
                'success': False,
                'vCar': 0.0,
                'aSteer': 0.0,
                'aSideslip': 0.0
            }
    except Exception as e:
        logger.error(f"Optimization error: {str(e)}")
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
        
        if result['success']:
            logger.info(f"Optimized vCar: {result['vCar']:.2f} m/s for curvature={curvature:.4f}")
            pointSpeeds.append(result['vCar'])
        else:
            logger.warning(f"Could not find equilibrium state for curvature={curvature:.4f}")
            pointSpeeds.append(0.0)
            
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

    # Compute lap time and collect acceleration traces
    lapTime = 0.0
    gLatChannel = []
    gLongChannel = []
    for i in range(1, len(track.points)):
        ds = track.points[i].distance - track.points[i-1].distance
        vPrev = finalSpeeds[i-1]
        vCurr = finalSpeeds[i]
        vAvg = (vCurr + vPrev) / 2
        # Longitudinal acceleration (finite difference)
        if ds > 0:
            gLong = ((vCurr**2 - vPrev**2) / (2 * ds)) / 9.81
        else:
            gLong = 0.0
        gLongChannel.append(gLong)
        # Lateral acceleration (from curvature)
        gLat = vCurr**2 * track.points[i].curvature / 9.81
        gLatChannel.append(gLat)
        if vAvg > 0:
            dt = ds / vAvg
            lapTime += dt
    logger.info(f"Estimated lap time: {lapTime:.2f} seconds")
    logger.info("Lap time simulation completed.")

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

    # Show all plots at once
    plt.show()