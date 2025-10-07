import numpy as np
import logging
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
        return np.inf
    
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

    maxIterations = 500
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

def initLapTimeSimulation(track, vehicle, config) -> None:
    """Initialize and run a lap time simulation."""
    logger.info("Starting lap time simulation...")
    # For each track point, optimize speed
    speeds = []
    for point in track.points[:10]:  # Test with first 10 points for now
        logger.info(f"Optimizing speed at distance={point.distance:.1f}m, curvature={point.curvature:.4f}")
        vCarLimit = optimiseSpeedAtPoint(point.curvature, vehicle, config)
        speeds.append(vCarLimit)
        logger.info(f"  -> Max speed: {vCarLimit:.2f} m/s")
    
    logger.info("Lap time simulation complete.")
    logger.info(f"Speed profile (first 10 points): {speeds}")
