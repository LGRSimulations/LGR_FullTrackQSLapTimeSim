import numpy as np
import logging
from .vehicleDynamics import find_vehicle_state_at_point

logger = logging.getLogger(__name__)


def _get_model_variant(config):
    """Resolve model variant from config with baseline-safe defaults."""
    ab_cfg = config.get('ab_testing', {}) if isinstance(config, dict) else {}
    variant = ab_cfg.get('model_variant', config.get('model_variant', 'baseline')) if isinstance(config, dict) else 'baseline'
    return str(variant).strip().lower()


def _is_b1_variant(config):
    variant = _get_model_variant(config)
    return variant in {'b1', 'load_transfer', 'dynamic_normal_load'}


def _get_straight_speed_cap(config):
    """Resolve straight-line speed cap (m/s) for near-zero curvature segments."""
    solver_cfg = config.get('solver', {}) if isinstance(config, dict) else {}
    cap = solver_cfg.get('straight_line_speed_cap_mps', 200.0)
    try:
        return float(cap)
    except (TypeError, ValueError):
        return 200.0


def _get_use_rollover_speed_cap(config):
    solver_cfg = config.get('solver', {}) if isinstance(config, dict) else {}
    return bool(solver_cfg.get('use_rollover_speed_cap', True))


def _compute_normal_loads_for_longitudinal(vehicle, v_car, a_long, config):
    """
    Compute per-tyre normal loads for front and rear axles.

    Baseline: static normal load per tyre.
    B1: include speed-dependent downforce and quasi-static longitudinal transfer.
    """
    base = vehicle.compute_static_normal_load()
    if not _is_b1_variant(config):
        return {
            'front_per_tyre': base,
            'rear_per_tyre': base,
            'mean_per_tyre': base,
            'total_normal_load': base * 4.0,
        }

    m = vehicle.params.mass
    g = 9.81
    h = vehicle.params.cog_z
    L = max(vehicle.params.wheelbase, 1e-6)
    front_frac = float(vehicle.params.cog_longitudinal_pos)
    front_frac = min(max(front_frac, 0.0), 1.0)
    rear_frac = 1.0 - front_frac

    f_down = max(0.0, vehicle.compute_downforce(v_car))
    total = m * g + f_down

    front_axle = total * front_frac
    rear_axle = total * rear_frac

    # Positive longitudinal acceleration shifts load rearward.
    delta_long = (m * a_long * h) / L
    front_axle -= delta_long
    rear_axle += delta_long

    # Clamp to keep solver numerically safe in extreme states.
    front_axle = max(0.0, front_axle)
    rear_axle = max(0.0, rear_axle)

    return {
        'front_per_tyre': front_axle / 2.0,
        'rear_per_tyre': rear_axle / 2.0,
        'mean_per_tyre': (front_axle + rear_axle) / 4.0,
        'total_normal_load': front_axle + rear_axle,
    }


def _compute_corner_normal_load_per_tyre(vehicle, v_car, config):
    """Cornering solve normal load proxy (baseline static, B1 includes aero load)."""
    base = vehicle.compute_static_normal_load()
    if not _is_b1_variant(config):
        return base

    m = vehicle.params.mass
    g = 9.81
    f_down = max(0.0, vehicle.compute_downforce(v_car))
    return (m * g + f_down) / 4.0

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
    solver_success = []
    fallback_used = []
    fallback_speed = []
    straight_speed_cap = _get_straight_speed_cap(config)
    use_rollover_speed_cap = _get_use_rollover_speed_cap(config)
    for point in track_points:
        curvature = point.curvature
        corner_load_per_tyre = _compute_corner_normal_load_per_tyre(vehicle, 20.0, config)
        result = find_vehicle_state_at_point(
            curvature,
            vehicle,
            normal_load_per_tyre=corner_load_per_tyre,
            straight_line_speed_cap=straight_speed_cap,
            use_rollover_speed_cap=use_rollover_speed_cap,
        )
        base_mu = getattr(vehicle.params, 'base_mu')
        if result['success']:
            logger.info(f"Optimized v_car: {result['v_car']:.2f} m/s for curvature={curvature:.4f}")
            point_speeds.append(result['v_car'])
            solver_success.append(True)
            fallback_used.append(False)
            fallback_speed.append(0.0)
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
            solver_success.append(False)
            fallback_used.append(True)
            fallback_speed.append(v_fallback)

    diagnostics = {
        'corner_solver_success': solver_success,
        'corner_fallback_used': fallback_used,
        'corner_fallback_speed': fallback_speed,
    }
    return point_speeds, diagnostics

def forward_pass(track, vehicle, point_speeds, config):
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
    limiting_modes = ['initial'] * n_points
    powertrain_forces = [0.0] * n_points
    tyre_limit_forces = [0.0] * n_points
    net_long_forces = [0.0] * n_points
    normal_load_per_tyre = [vehicle.compute_static_normal_load()] * n_points
    peak_slip_ratio = float(config.get('ab_testing', {}).get('peak_slip_ratio_accel', 12.0))

    for i in range(1, n_points):
        ds = track.points[i].distance - track.points[i-1].distance
        if ds <= 0:
            speeds[i] = speeds[i-1]
            limiting_modes[i] = 'degenerate_segment'
            continue
        v_prev = speeds[i-1]
        v_calc = max(v_prev, 1.0)
        gear_ratio = vehicle.select_optimal_gear(v_calc)

        # Powertrain-limited wheel force at the contact patch.
        rpm = vehicle.speed_to_rpm(v_calc, gear_ratio)
        wheel_torque = vehicle.compute_wheel_torque(rpm, gear_ratio, throttle=1.0)
        f_traction_powertrain = wheel_torque / vehicle.params.wheel_radius

        # Estimate prior-step longitudinal acceleration for B1 load transfer coupling.
        a_long_est = 0.0
        if ds > 0.0:
            a_long_est = (v_calc**2 - v_prev**2) / (2.0 * ds)

        load_state = _compute_normal_loads_for_longitudinal(vehicle, v_calc, a_long_est, config)
        front_load = load_state['front_per_tyre']
        rear_load = load_state['rear_per_tyre']
        normal_load_per_tyre[i] = load_state['mean_per_tyre']

        # Tyre-limited wheel force at fixed peak slip ratio with variant-aware normal load.
        mu_scale = float(vehicle.params.base_mu) / max(getattr(vehicle, 'base_mu_reference', 1.0), 1e-6)
        f_drive_front = vehicle.tyre_model.get_longitudinal_force(
            slip_ratio=peak_slip_ratio,
            normal_load=front_load
        ) * mu_scale
        f_drive_per_tyre = vehicle.tyre_model.get_longitudinal_force(
            slip_ratio=peak_slip_ratio,
            normal_load=rear_load
        ) * mu_scale
        f_traction_tyre_limit = abs(f_drive_front) * 2.0 + abs(f_drive_per_tyre) * 2.0

        # Net longitudinal force after tyre slip limit and drag.
        f_traction = min(f_traction_powertrain, f_traction_tyre_limit)
        f_drag = vehicle.compute_aero_drag(v_calc)
        f_x = f_traction - f_drag

        powertrain_forces[i] = f_traction_powertrain
        tyre_limit_forces[i] = f_traction_tyre_limit
        net_long_forces[i] = f_x
        if f_traction_powertrain < f_traction_tyre_limit:
            limiting_modes[i] = 'power_limited'
        else:
            limiting_modes[i] = 'traction_limited'

        a_lon = f_x / vehicle.params.mass
        v_pred_squared = v_prev**2 + 2 * a_lon * ds
        if v_pred_squared < 0:
            v_pred = v_prev
        else:
            v_pred = np.sqrt(v_pred_squared)
        if v_pred > point_speeds[i]:
            limiting_modes[i] = 'corner_capped'
        v_pred = min(v_pred, point_speeds[i])
        speeds[i] = v_pred
    diagnostics = {
        'forward_limiting_mode': limiting_modes,
        'forward_powertrain_force': powertrain_forces,
        'forward_tyre_force_limit': tyre_limit_forces,
        'forward_net_long_force': net_long_forces,
        'forward_normal_load_per_tyre': normal_load_per_tyre,
    }
    return speeds, diagnostics

def backward_pass(track, vehicle, point_speeds, config):
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
    limiting_modes = ['terminal'] * n_points
    brake_force_limits = [0.0] * n_points
    brake_decel_limits = [0.0] * n_points
    normal_load_per_tyre = [vehicle.compute_static_normal_load()] * n_points
    peak_slip_ratio = float(config.get('ab_testing', {}).get('peak_slip_ratio_brake', 12.0))

    for i in range(n_points-2, -1, -1):
        ds = track.points[i+1].distance - track.points[i].distance
        if ds <= 0:
            speeds[i] = speeds[i+1]
            limiting_modes[i] = 'degenerate_segment'
            continue
        v_next = speeds[i+1]

        # Positive a_long here is deceleration demand in backward pass approximation.
        a_long_est = (max(v_next, 0.0) ** 2) / (2.0 * max(ds, 1e-6))
        load_state = _compute_normal_loads_for_longitudinal(vehicle, v_next, -a_long_est, config)
        front_load = load_state['front_per_tyre']
        rear_load = load_state['rear_per_tyre']
        normal_load_per_tyre[i] = load_state['mean_per_tyre']

        mu_scale = float(vehicle.params.base_mu) / max(getattr(vehicle, 'base_mu_reference', 1.0), 1e-6)
        f_brake_front = vehicle.tyre_model.get_longitudinal_force(
            slip_ratio=-peak_slip_ratio,
            normal_load=front_load
        ) * mu_scale
        f_brake_rear = vehicle.tyre_model.get_longitudinal_force(
            slip_ratio=-peak_slip_ratio,
            normal_load=rear_load
        ) * mu_scale
        f_brake_total = abs(f_brake_front) * 2 + abs(f_brake_rear) * 2

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
        brake_force_limits[i] = f_brake_total
        brake_decel_limits[i] = a_brake
        if a_lat_clamped >= a_limit * 0.99:
            limiting_modes[i] = 'lateral_saturated'
        else:
            limiting_modes[i] = 'brake_limited'

        v_brake_squared = v_next**2 + 2 * a_brake * ds
        if v_brake_squared < 0:
            v_brake = v_next
        else:
            v_brake = np.sqrt(v_brake_squared)
        if v_brake > point_speeds[i]:
            limiting_modes[i] = 'corner_capped'
        speeds[i] = min(v_brake, point_speeds[i])

    diagnostics = {
        'backward_limiting_mode': limiting_modes,
        'backward_brake_force_limit': brake_force_limits,
        'backward_brake_decel_limit': brake_decel_limits,
        'backward_normal_load_per_tyre': normal_load_per_tyre,
    }
    return speeds, diagnostics

def compute_speed_profile(track, vehicle, config):
    """
    Compute final speed profile using forward and backward passes for all track points.
    Parallelises the optimisation of cornering speeds at each point.
    """
    # Pass 1: Find ultimate speeds possible given vehicle model
    point_speeds, corner_diag = optimise_speed_at_points(track.points, vehicle, config)
    # Pass 2: Forward pass propagating accel limits
    forward_speeds, forward_diag = forward_pass(track, vehicle, point_speeds, config)
    # Pass 3: Backward pass propagating decel limits
    backward_speeds, backward_diag = backward_pass(track, vehicle, point_speeds, config)

    # Take minimum of forward and backward speeds to get final speed profile
    final_speeds = np.minimum(forward_speeds, backward_speeds)
    diagnostics = {
        **corner_diag,
        **forward_diag,
        **backward_diag,
        'model_variant': _get_model_variant(config),
    }
    return final_speeds, point_speeds, diagnostics
