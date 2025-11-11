import numpy as np
import logging
from scipy.optimize import minimize
from prettyGraphs.lapTimeResults import plot_lap_time_results, LapTimeResults

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
    
    # Define constraint function for yaw moment equilibrium
    def constraint_yaw_moment(x):
        """
        Ensure that the yaw moment is close to zero. We use a tolerance of 50 Nm.
        """
        v_car, a_steer, a_sideslip = x
        result = evaluate_vehicle_state(v_car, a_steer, a_sideslip, curvature, vehicle, debug_mode)
        tolerance = 50.0
        return tolerance - abs(result['m_z'])
    
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
        return 4 - abs(result['g_lat']) / 9.81

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
        {'type': 'ineq', 'fun': constraint_yaw_moment},
        {'type': 'ineq', 'fun': constraint_g_lat},
        {'type': 'ineq', 'fun': constraint_max_g_lat}
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
    
def optimise_speed_at_points(track_points, vehicle, config):
    """
    Find maximum v_car at multiple track points where equilibrium states exist.
    Uses direct constrained optimization for each point.
    Effectively gives us max cornering speeds given our vehicle model.
    Args:
        track_points: List of track points with curvature data
        vehicle: Vehicle object
        config: Config dict
    Returns:
        point_speeds: List of maximum speeds for each point (m/s)
    """
    point_speeds = []
    for point in track_points:
        curvature = point.curvature
        result = find_vehicle_state_at_point(curvature, vehicle)
        base_mu = getattr(vehicle.tyre_model, 'base_mu', getattr(vehicle.tyre_model, 'base_mu', None))
        if result['success']:
            logger.info(f"Optimized v_car: {result['v_car']:.2f} m/s for curvature={curvature:.4f}")
            point_speeds.append(result['v_car'])
        else:
            logger.warning(f"Could not find equilibrium state for curvature={curvature:.4f}")
            logger.warning(f"This is point at coordinates x={point.x}, y={point.y}, z={point.z}")
            g = 9.81
            if abs(curvature) > 1e-6:
                v_fallback = np.sqrt(base_mu * g / abs(curvature))
            else:
                v_fallback = 200.0
            logger.warning(f"Fallback: using v_fallback={v_fallback:.2f} m/s based on base_mu={base_mu}")
            point_speeds.append(v_fallback)
    return point_speeds

def forward_pass(track, vehicle, point_speeds):
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
    n_points = len(track.points)
    speeds = np.zeros(n_points)
    speeds[0] = point_speeds[0]  # start at first point speed
    for i in range(1, n_points):
        ds = track.points[i].distance - track.points[i-1].distance
        if ds <= 0:
            speeds[i] = speeds[i-1]
            continue
        v_prev = speeds[i-1]
        v_calc = max(v_prev, 1.0)
        gear_ratio = vehicle.select_optimal_gear(v_calc)
        f_x = vehicle.compute_longitudinal_force(v_calc, gear_ratio, throttle=1.0)
        a_lon = f_x / vehicle.params.mass
        v_pred_squared = v_prev**2 + 2 * a_lon * ds
        if v_pred_squared < 0:
            v_pred = v_prev
        else:
            v_pred = np.sqrt(v_pred_squared)
        v_pred = min(v_pred, point_speeds[i])
        speeds[i] = v_pred
    return speeds

def backward_pass(track, vehicle, point_speeds):
    """
    Backward pass: braking-limited speed profile using tyre model.
    Propagates speeds backward from end to start. At each segment:
    - Compute maximum braking force from tyre longitudinal grip (using DX_LUT)
    - Calculate maximum deceleration: a_brake = f_brake / mass
    - Work backwards: v_i = sqrt(v_{i+1}^2 + 2 * a_brake * ds)
    - Enforce v_i <= point_speeds[i] (corner limit)
    Uses all track points.
    """
    n_points = len(track.points)
    speeds = np.zeros(n_points)
    speeds[-1] = point_speeds[-1]
    for i in range(n_points-2, -1, -1):
        ds = track.points[i+1].distance - track.points[i].distance
        if ds <= 0:
            speeds[i] = speeds[i+1]
            continue
        v_next = speeds[i+1]
        normal_load_per_tyre = vehicle.compute_static_normal_load()
        peak_slip_ratio = 12.0
        f_brake_per_tyre = vehicle.tyre_model.get_longitudinal_force(
            slip_ratio=-peak_slip_ratio,
            normal_load=normal_load_per_tyre
        )
        f_brake_total = abs(f_brake_per_tyre) * 4
        v_avg = v_next
        f_drag = vehicle.compute_aero_drag(v_avg)
        f_brake_total_with_drag = f_brake_total + f_drag
        a_brake = f_brake_total_with_drag / vehicle.params.mass
        v_brake_squared = v_next**2 + 2 * a_brake * ds
        if v_brake_squared < 0:
            v_brake = v_next
        else:
            v_brake = np.sqrt(v_brake_squared)
        speeds[i] = min(v_brake, point_speeds[i])
    return speeds

def compute_speed_profile(track, vehicle, config):
    """
    Compute final speed profile using forward and backward passes for all track points.
    Parallelises the optimisation of cornering speeds at each point.
    """
    point_speeds = optimise_speed_at_points(track.points, vehicle, config)
    forward_speeds = forward_pass(track, vehicle, point_speeds)
    backward_speeds = backward_pass(track, vehicle, point_speeds)
    final_speeds = np.minimum(forward_speeds, backward_speeds)
    return final_speeds, point_speeds

def run_lap_time_simulation(track, vehicle, config) -> None:
    """Initialize and run a lap time simulation for the whole track."""
    logger.info("Starting lap time simulation...")
    final_speeds, corner_speeds = compute_speed_profile(track, vehicle, config)
    lap_time = 0.0
    g_lat_channel = []
    g_long_channel = []
    normal_load_per_tyre = []
    long_force_per_tyre = []
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
    )
    plot_lap_time_results(results)