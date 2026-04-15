import numpy as np
import logging
from .vehicleDynamics import find_vehicle_state_at_point

logger = logging.getLogger(__name__)

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
    for idx, point in enumerate(track_points):
        if idx == 0:
            point_speeds.append(0.01)
            continue
        else:
            curvature = point.curvature
            result = find_vehicle_state_at_point(curvature, vehicle)
            base_mu = getattr(vehicle.params, 'base_mu')
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
    Propagates speeds forward using available engine forces, capped by tyre
    longitudinal grip at a fixed wheel-slip operating point, minus drag.
    At each segment:
    - Select optimal gear for current speed
    - Compute available traction force from powertrain (F_x = T_wheel / r_wheel)
    - Cap traction by tyre longitudinal force at peak slip ratio
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

        # Powertrain-limited wheel force at the contact patch.
        rpm = vehicle.speed_to_rpm(v_calc, gear_ratio)
        wheel_torque = vehicle.compute_wheel_torque(rpm, gear_ratio, throttle=1.0)
        f_traction_powertrain = wheel_torque / vehicle.params.wheel_radius

        # Tyre-limited wheel force at a fixed peak slip ratio.
        normal_load_per_tyre = vehicle.compute_static_normal_load()
        peak_slip_ratio = 12.0 
        f_drive_per_tyre = vehicle.tyre_model.get_longitudinal_force(
            slip_ratio=peak_slip_ratio,
            normal_load=normal_load_per_tyre
        )
        f_traction_tyre_limit = abs(f_drive_per_tyre) * 4.0

        # Net longitudinal force after tyre slip limit and drag.
        f_traction = min(f_traction_powertrain, f_traction_tyre_limit)
        f_drag = vehicle.compute_aero_drag(v_calc)
        f_x = f_traction - f_drag

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
    Backward pass: braking-limited speed profile using tyre model with friction circle.
    Propagates speeds backward from end to start. At each segment:
    - Compute peak longitudinal deceleration from tyre grip (a_limit = F_brake / mass)
    - Compute lateral acceleration demand at this point: a_lat = v^2 * curvature
    - Apply friction circle to find available longitudinal deceleration:
        a_long_available = sqrt(max(a_limit^2 - a_lat^2, 0))
      This ensures combined lateral + longitudinal demand never exceeds tyre capacity.
    - Add aerodynamic drag contribution
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

        # Peak longitudinal deceleration available from tyres (m/s^2)
        a_limit = f_brake_total / vehicle.params.mass

        # Lateral acceleration demanded at this point by the corner geometry (m/s^2)
        curvature = abs(track.points[i].curvature)
        a_lat = v_next**2 * curvature

        # Friction circle: longitudinal budget is reduced by lateral demand.
        # a_long^2 + a_lat^2 <= a_limit^2  =>  a_long = sqrt(a_limit^2 - a_lat^2)
        a_lat_clamped = min(a_lat, a_limit)  # cannot exceed total limit
        a_brake = np.sqrt(max(a_limit**2 - a_lat_clamped**2, 0.0))

        # Add aerodynamic drag (acts as free braking, outside friction circle)
        f_drag = vehicle.compute_aero_drag(v_next)
        a_brake += f_drag / vehicle.params.mass

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
    # Pass 1: Find ultimate speeds possible given vehicle model
    point_speeds = optimise_speed_at_points(track.points, vehicle, config)
    # Pass 2: Forward pass propagating accel limits
    forward_speeds = forward_pass(track, vehicle, point_speeds)
    # Pass 3: Backward pass propagating decel limits
    backward_speeds = backward_pass(track, vehicle, point_speeds)

    # Take minimum of forward and backward speeds to get final speed profile
    final_speeds = np.minimum(forward_speeds, backward_speeds)
    return final_speeds, point_speeds
