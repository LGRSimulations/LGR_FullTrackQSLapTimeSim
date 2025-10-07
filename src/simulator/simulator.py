import numpy as np
import logging
import concurrent.futures
import matplotlib.pyplot as plt
import seaborn as sns

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
    if abs(curvature) < 1e-6:  # straight line
        return np.float64(70.0)    # arbitrary high speed on straights
    
    # calculate vCarMax based on lateral acceleration and curvature
    vCarMax = np.sqrt(abs(gLat) / abs(curvature))   # m/s
    return vCarMax

def iterateSteeringSideslip(vCar: float, curvature: float, vehicle, aSteerRange: np.ndarray, aSideslipRange: np.ndarray):
    """
    f1: For a given vCar and track curvature, iterate over aSteer and aSideslip
    to find states where yaw moment = 0.
    
    Args:
        vCar: Current vehicle speed in m/s
        curvature: Track curvature in 1/m
        vehicle: Vehicle object
        aSteerRange: Array of steering angles to try (degrees)
        aSideSlipRange: Array of sideslip angles to try (degrees)
        
    Returns:
        gLat: Lateral acceleration at the valid state (or None if no valid state found)
    """

    validStates = []

    for aSteer in aSteerRange:
        for aSideslip in aSideslipRange:
            # Simplified bicycle model: compute slip angles at front and rear
            # slip_angle_front = aSteer - aSideslip
            # slip_angle_rear = -aSideslip
            # (This is a simplification; a full bicycle model would include more dynamics)
            
            slipAngleFront = aSteer - aSideslip
            slipAngleRear = -aSideslip

            # Get tyre forces 
            F_front, F_rear = vehicle.computeTyreForces(slipAngleFront, slipAngleRear)

            # Compute yaw moment
            M_z = vehicle.computeYawMoment(F_front, F_rear, aSteer)

            # Check if yaw moment is zero (steady state)
            if abs(M_z) < 1e-2:  # small threshold for numerical stability
                # Compute lateral acceleration
                gLat = vehicle.computeLateralAcceleration(F_front, F_rear, vCar)
                validStates.append((aSteer, aSideslip, gLat, M_z))
    if not validStates:
        return None  # No valid state found
    # Return the gLat of the first valid state found
    return validStates[0][2]

def optimiseSpeedAtPoint(curvature: float, vehicle, config):
    """
    Main optimisation loop for a single track point.
    
    1. Guess vCar
    2. Iterate over aSteer & aSideslip, filter for yaw moment = 0
    3. If valid state found, compute gLat and check if under vCarMax
    4. If yes, increase vCar and try again
    5. Else return previous vCar as limit
    
    Args:
        curvature: Track curvature at this point (1/m)
        vehicle: Vehicle object
        config: Config dict
        
    Returns:
        vCarLimit: Maximum speed at this point (m/s)
    """
    
    # Initial guess
    vCar = 0.5     # m/s
    vCarStep = 0.5  # m/s increment
    vCarPrev = 0.0
    
    # Define ranges for aSteer and aSideslip (degrees)
    aSteerRange = np.linspace(-10, 10, 21)  # degrees
    aSideslipRange = np.linspace(-5, 5, 11)  # degrees

    maxIterations = 2000
    iteration = 0

    while iteration < maxIterations:
        # f1: Try to find a valid state at this vCar
        gLat = iterateSteeringSideslip(vCar, curvature, vehicle, aSteerRange, aSideslipRange)
        
        if gLat is None:
            # No valid state found, speed too high
            logger.debug(f"No valid state at vCar={vCar:.2f} m/s")
            return vCarPrev
        # f2: Compute theoretical vCarMax
        vCarMax = computeVCarMax(gLat, curvature)

        # Check if current vCar under the limit
        if vCar < vCarMax:
            # Increase speed and try again
            vCarPrev = vCar
            vCar += vCarStep
            logger.debug(f"vCar={vCar:.2f} m/s under limit {vCarMax:.2f} m/s, increasing...")
        else:
            # Reached limit
            logger.debug(f"vCar={vCar:.2f} m/s reached limit {vCarMax:.2f} m/s")
            return vCarPrev
        iteration += 1

    logger.warning("Max iterations reached without convergence at curvature={curvature:.4f}")
    return vCarPrev

def forwardPass(track, vehicle, pointSpeeds):
    """
    Forward pass: acceleration-limited speed profile.
    Uses all track points.
    """
    n_points = len(track.points)
    speeds = np.zeros(n_points)
    speeds[0] = pointSpeeds[0]  # start at first point speed

    for i in range(1, n_points):
        # Distance to next point
        ds = track.points[i].distance - track.points[i-1].distance

        # Maximum acceleration (from vehicle limits)
        a_max = vehicle.params.maxGLongAccel * 9.81  # m/s^2

        # Maximum speed we can reach accelerating from previous point
        v_accel = np.sqrt(speeds[i-1]**2 + 2 * a_max * ds)

        # Take minimum of acceleration-limited and corner-limited speed
        speeds[i] = min(v_accel, pointSpeeds[i])

    return speeds

def backwardPass(track, vehicle, pointSpeeds):
    """
    Backward pass: braking-limited speed profile.
    Uses all track points.
    """
    n_points = len(track.points)
    speeds = np.zeros(n_points)
    speeds[-1] = pointSpeeds[-1]  # end at last point speed

    for i in range(n_points-2, -1, -1):
        # Distance to next point
        ds = track.points[i+1].distance - track.points[i].distance

        # Maximum deceleration (from vehicle limits)
        aBrake = vehicle.params.maxGLongBrake * 9.81  # m/s^2

        # Maximum speed we can reach braking from next point
        vBrake = np.sqrt(speeds[i+1]**2 + 2 * aBrake * ds)

        # Take minimum of braking-limited and corner-limited speed
        speeds[i] = min(vBrake, pointSpeeds[i])

    return speeds

def computeSpeedProfile(track, vehicle, config):
    """
    Compute final speed profile using forward and backward passes for all track points.
    Parallelises the optimisation of cornering speeds at each point.
    """
    # Step 1: For each track point, optimise max cornering speed given vehicle model
    def optimiseAtPoints(point):
        logger.info(f"Optimizing speed at distance={point.distance:.1f}m, curvature={point.curvature:.4f}")
        vCarLimit = optimiseSpeedAtPoint(point.curvature, vehicle, config)
        logger.info(f"  -> Max speed: {vCarLimit:.2f} m/s")
        return vCarLimit

    with concurrent.futures.ThreadPoolExecutor() as executor:
        pointSpeeds = list(executor.map(optimiseAtPoints, track.points))
    
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

    # Compute lap time
    lapTime = 0.0
    for i in range(1, len(track.points)):
        ds = track.points[i].distance - track.points[i-1].distance
        v_avg = (finalSpeeds[i] + finalSpeeds[i-1]) / 2
        if v_avg > 0:
            dt = ds / v_avg
            lapTime += dt
    logger.info(f"Estimated lap time: {lapTime:.2f} seconds")
    logger.info("Lap time simulation completed.")

    # Plot results
    sns.set_theme(style="darkgrid", context="notebook")
    distances = [p.distance for p in track.points]
    plt.figure(figsize=(12, 6))
    plt.plot(distances, finalSpeeds, label='vCar (Simulated)', linewidth=2)
    plt.plot(distances, cornerSpeeds, '--', label='Theoretical Limit', alpha=0.7)
    plt.xlabel('Distance along track (m)')
    plt.ylabel('Speed (m/s)')
    plt.title(f'Lap Time: {lapTime:.2f} s\nSpeed Profile vs Theoretical Limit')
    plt.legend()
    plt.tight_layout()
    plt.show()
