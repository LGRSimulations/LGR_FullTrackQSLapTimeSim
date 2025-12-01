import numpy as np
import logging
from scipy.optimize import minimize

logger = logging.getLogger(__name__)

def compute_v_car_max(g_lat: float, curvature: float) -> float:
    """
    Compute maximum theoretical speed for a given lateral acceleration and track curvature.
    v_max = sqrt(a_lat / curvature)
    Args:
        g_lat: Lateral acceleration in m/s^2
        curvature: Track curvature in 1/m
    Returns:
        Maximum speed in m/s
    """
    v_car_max = np.sqrt(abs(g_lat) / abs(curvature))   # m/s
    return v_car_max

def estimate_roll_angle(v_car: float, a_steer: float, a_sideslip: float, curvature: float, vehicle, debug_mode=False):
    """
    Conservative roll angle estimate (rad) from lateral load transfer:
      roll_angle = roll_moment / roll_stiffness
    Returns (roll_angle_rad, margin) where margin = max_roll_angle_rad - abs(roll_angle_rad).
    """
    params = getattr(vehicle, 'params', None)
    mass = getattr(params, 'mass', 1500.0)
    cog_z = getattr(params, 'cog_z', getattr(params, 'cog_z'))
    roll_stiffness = getattr(params, 'roll_stiffness')  # Nm/rad total
    max_roll_angle_deg = getattr(params, 'max_roll_angle_deg')  # degrees allowed before fail
    max_roll_angle_rad = np.deg2rad(max_roll_angle_deg)

    # lateral acceleration (use curvature if present)
    try:
        g_lat = (v_car**2) * curvature
    except Exception:
        g_lat = 0.0
    g_lat = float(abs(g_lat))

    F_lat = mass * g_lat
    roll_moment = F_lat * cog_z
    roll_angle = roll_moment / max(1e-12, roll_stiffness)

    margin = max_roll_angle_rad - abs(roll_angle)
    if debug_mode:
        print(f"[DEBUG][roll] v={v_car:.2f} g_lat={g_lat:.2f} roll_angle={roll_angle:.4f} rad "
              f"max={max_roll_angle_rad:.4f} margin={margin:.4f}")
    return roll_angle, margin
    
def estimate_effective_cog_z(v_car: float, a_steer: float, a_sideslip: float, curvature: float, vehicle, debug_mode) -> tuple:
    """
    Estimate effective CoG z (meters) after roll and aero-induced suspension compression.
    Returns (feasible: bool, cog_z_est: float, margin: float) where margin = max_CoG_z - cog_z_est.
    Uses vehicle.params.* when available, otherwise falls back to conservative defaults.
    """

    params = getattr(vehicle, 'params', None)
    mass = getattr(params, 'mass')               # kg
    roll_stiffness = getattr(params, 'roll_stiffness') # Nm/rad total      # kg/m^3
    max_CoG_z = getattr(params, 'max_cog_z')  
    cog_z = getattr(params, 'cog_z')                     # m
    track = getattr(params, 'rear_track_width')               # m

    # compute lateral acceleration (m/s^2) (use curvature based if available)
    try:
        g_lat = (v_car**2) * curvature
    except Exception:
        g_lat = 0.0
    g_lat = float(abs(g_lat))

    # lateral force and roll moment
    F_lat = mass * g_lat                    # N
    roll_moment = F_lat * cog_z             # Nm (approx, lever = cog_z)

    # approximate roll angle (rad) and resulting vertical compression (heuristic)
    roll_angle = roll_moment / max(1e-12, roll_stiffness)   # rad
    # vertical displacement due to roll: approximate using small-angle geometry
    # assume one side compresses roughly by track/2 * sin(roll_angle) -> use absolute value
    delta_h_roll = abs((track / 2.0) * np.sin(roll_angle))

    # total estimated lowering of CoG
    cog_z_est = cog_z - (delta_h_roll)
    # safety clamp (don't go negative)
    cog_z_est = float(max(0.0, cog_z_est))

    margin = max_CoG_z - cog_z_est
    feasible = (cog_z_est <= max_CoG_z)

    return feasible, cog_z_est, margin

def evaluate_vehicle_state(v_car: float, a_steer: float, a_sideslip: float, curvature: float, vehicle, debug_mode) -> dict:
    """
    Evaluate vehicle state for given v_car, a_steer, a_sideslip, and track curvature.
    Args:
        v_car: Vehicle speed (m/s)
        a_steer: Steering angle (degrees)
        a_sideslip: Sideslip angle (degrees)
        curvature: Track curvature (1/m)
        vehicle: Vehicle object
        debug_mode: Debug mode flag
    Returns:
        dict: {'g_lat': float, 'm_z': float, 'is_valid': bool, 'f_front': float, 'f_rear': float}
    """
    slip_angle_front = a_steer - a_sideslip
    slip_angle_rear = -a_sideslip
    f_front, f_rear = vehicle.compute_tyre_forces(slip_angle_front, slip_angle_rear)
    m_z = vehicle.compute_yaw_moment(f_front, f_rear, a_steer)
    g_lat = vehicle.compute_lateral_acceleration(f_front, f_rear, v_car)
    is_valid = abs(m_z) < 50.0
    if debug_mode:
        print(f"[DEBUG] variables: v_car={v_car:.2f}, g_lat={g_lat:.2f}, m_z={m_z:.2f}, is_valid={is_valid}, f_front={f_front:.2f}, f_rear={f_rear:.2f}")
    return {
        'g_lat': g_lat,
        'm_z': m_z,
        'is_valid': is_valid,
        'f_front': f_front,
        'f_rear': f_rear
    }

def find_vehicle_state_at_point(curvature: float, vehicle):
    """
    Finds the equilibrium vehicle state (speed, steering angle, sideslip) that satisfies yaw moment equilibrium
    and maximizes vehicle speed for a given track curvature using constrained optimization.

    The optimization seeks the highest possible speed (v_car) at which the vehicle can negotiate a curve of given curvature,
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
            'v_car': float,           # Maximum feasible speed at this curvature (m/s)
            'a_steer': float,         # Steering angle (degrees)
            'a_sideslip': float       # Sideslip angle (degrees)
        }
    """
    # Handle straight sections (zero curvature)
    if abs(curvature) < 1e-3:
        # For straight sections, maximum speed is limited by drag/power
        # For now, return a high speed with zero steering
        return {
            'success': True,
            'v_car': 200.0,  # High speed for straights (adjust based on power limits)
            'a_steer': 0.0,
            'a_sideslip': 0.0
        }

    # For debug: store first and last tried parameters and results
    debug_mode = getattr(vehicle, 'config', {}).get('debug_mode', False)
    debug_info = {'first_x': None, 'first_result': None, 'last_x': None, 'last_result': None}

    def objective(x):
        v_car, a_steer, a_sideslip = x
        result = evaluate_vehicle_state(v_car, a_steer, a_sideslip, curvature, vehicle, debug_mode)
        penalty = abs(result['m_z']) / 50.0 # Penalty for yaw moment deviation
        if debug_mode:
            if debug_info['first_x'] is None:
                debug_info['first_x'] = x.copy() if hasattr(x, 'copy') else list(x)
                debug_info['first_result'] = result.copy() if hasattr(result, 'copy') else dict(result)
            debug_info['last_x'] = x.copy() if hasattr(x, 'copy') else list(x)
            debug_info['last_result'] = result.copy() if hasattr(result, 'copy') else dict(result)
        return -v_car + penalty


    # Define constraint function for roll angle
    def constraint_roll_angle(x):
        """Inequality constraint: max_roll_angle - abs(roll_angle) >= 0"""
        v_car, a_steer, a_sideslip = x
        roll_angle, margin = estimate_roll_angle(v_car, a_steer, a_sideslip, curvature, vehicle, debug_mode)
        if not np.isfinite(roll_angle):
            return -1e6
        return margin

    # Define constraint function for CoG z-axis
    def constraint_CoG_z_axis(x):
            """
            Ensure that the vertical height of the center of gravity remains within physical limits.
            Uses estimate_effective_cog_z to compute an estimated CoG z for the current state.
            """
            v_car, a_steer, a_sideslip = x
            feasible, cog_z_est, margin = estimate_effective_cog_z(v_car, a_steer, a_sideslip, curvature, vehicle, debug_mode)
            # constraint must be >= 0 for 'ineq' type: return margin (max_CoG_z - cog_z_est)
            # If estimate returned NaN or invalid, return large negative so optimizer treats it infeasible
            if not np.isfinite(cog_z_est):
                return -1e6
            return margin

    # Define constraint function for lateral acceleration vs. curvature
    def constraint_g_lat(x):
        """
        Ensure that the lateral acceleration corresponds to a feasible maximum speed for the given curvature.
        """
        v_car, a_steer, a_sideslip = x
        result = evaluate_vehicle_state(v_car, a_steer, a_sideslip, curvature, vehicle, debug_mode)
        v_car_max = compute_v_car_max(result['g_lat'], curvature)
        return v_car_max - v_car

    def constraint_max_g_lat(x):
        """
        Ensure that the lateral acceleration does not exceed a maximum limit, in this case 2G.
        """
        v_car, a_steer, a_sideslip = x
        result = evaluate_vehicle_state(v_car, a_steer, a_sideslip, curvature, vehicle, debug_mode)
        return 2 - abs(result['g_lat']) / 9.81

    # Initial guess based on simple bicycle model
    # Estimate initial steering angle from curvature and wheelbase
    L = vehicle.params.wheelbase
    initial_steer_angle = np.degrees(abs(curvature) * L)      # Simple Ackermann (steer_angle (radians) = curvature * L)
    initial_steer_angle = np.clip(initial_steer_angle, 0.1, 8.0)    # Reasonable range

    # Initial guess: conservative speed, estimated steering, small sideslip
    initial_guess = [5.0, initial_steer_angle, 0.5]  # [v_car, steer_angle, sideslip_angle]

    # Define bounds for optimization variables
    bounds = [
        (1.0, 150.0),     # v_car between 1 and 150 m/s
        (-30.0, 30.0),    # a_steer between -30 and 30 degrees
        (-5.0, 5.0)       # a_sideslip between -5 and 5 degrees
    ]

    # Define constraints
    constraints = [
        {'type': 'ineq', 'fun': constraint_CoG_z_axis},
        {'type': 'ineq', 'fun': constraint_g_lat},
        {'type': 'ineq', 'fun': constraint_max_g_lat},
        {'type': 'ineq', 'fun': constraint_roll_angle}
    ]

    # Run optimization
    try:
        result = minimize(
            objective, 
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            constraints=constraints,
            options={'disp': False, 'maxiter': 2000}
        )

        if result.success:
            v_car, steer_angle, sideslip_angle = result.x
            logger.info(f"Optimization successful: v_car={v_car:.2f}, steer_angle={steer_angle:.2f}, sideslip_angle={sideslip_angle:.2f}")
            return {
                'success': True,
                'v_car': v_car,
                'steer_angle': steer_angle,
                'sideslip_angle': sideslip_angle
            }
        else:
            logger.warning(f"Optimization failed: {result.message}")
            if debug_mode:
                print("[DEBUG] Optimization initial guess:", debug_info['first_x'])
                print("[DEBUG] Initial evaluate_vehicle_state:", debug_info['first_result'])
                print("[DEBUG] Optimization last tried x:", debug_info['last_x'])
                print("[DEBUG] Last evaluate_vehicle_state:", debug_info['last_result'])
            return {
                'success': False,
                'v_car': 0.0,
                'steer_angle': 0.0,
                'sideslip_angle': 0.0
            }
    except Exception as e:
        logger.error(f"Optimization error: {str(e)}")
        if debug_mode:
            print("[DEBUG] Exception during optimization:", str(e))
            print("[DEBUG] Optimization initial guess:", debug_info['first_x'])
            print("[DEBUG] Initial evaluate_vehicle_state:", debug_info['first_result'])
            print("[DEBUG] Optimization last tried x:", debug_info['last_x'])
            print("[DEBUG] Last evaluate_vehicle_state:", debug_info['last_result'])
        return {
            'success': False,
            'v_car': 0.0,
            'steer_angle': 0.0,
            'sideslip_angle': 0.0
        }
