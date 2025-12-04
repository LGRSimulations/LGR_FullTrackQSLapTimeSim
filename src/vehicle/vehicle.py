import logging
import numpy as np
from dataclasses import dataclass
import pandas as pd
from scipy.interpolate import interp1d

logger = logging.getLogger(__name__)

@dataclass
class vehicle_parameters:
    """Core vehicle parameters for point mass simulation."""
    name: str
    mass: float  # kg
    base_mu: float  # dimensionless (base coefficient of friction for tyres)
    frontal_area: float  # m²
    drag_coefficient: float  # dimensionless
    downforce_coefficient: float  # dimensionless (L/D ratio typically)
    aero_centre_of_pressure: float  # m from front axle
    wheelbase: float  # m
    front_track_width: float  # m
    rear_track_width: float  # m
    cog_z: float  # m
    max_cog_z: float  # m
    max_roll_angle_deg: float  # degrees
    cog_longitudinal_pos: float  # fraction of wheelbase from front axle (0 to 1)
    wheel_radius: float  # m (effective rolling radius)
    final_drive_ratio: float  # dimensionless (final drive gear ratio)
    gear_ratios: list  # dimensionless (gear ratios for each gear)
    transmission_efficiency: float  # dimensionless (0-1, mechanical efficiency)
    roll_stiffness: float  # Nm/deg
    suspension_stiffness: float  # N/m
    damping_coefficient: float  # Ns/m


class PowerUnit:
    """
    Power unit model with torque/power curves and efficiency maps.
    Handles both ICE and electric powertrains.
    """
    
    def __init__(self, power_data: pd.DataFrame):
        """
        Initialize power unit.
        
        Args:
            power_data: DataFrame with columns ['Motor Speed [RPM]', 'Continuous Power [kW]', 'Peak Power [kW]']
        """
        self.power_data = self.setup_from_data(power_data)
        if self.power_data is None:
            raise ValueError("power_data must be a pandas DataFrame and cannot be None.")

    def setup_from_data(self, df: pd.DataFrame):
        """Setup power unit from DataFrame."""
        return df
    
class tyre_model:
    """
    tyre model for grip calculations.
    Use interpolated lookup tables for tyre forces.
    """
    
    def __init__(self, tyre_data_lat: pd.DataFrame, tyre_data_long: pd.DataFrame):
        """
        Initialize tyre model.
        
        Args:
            tyre_data_lat: DataFrame with lateral tyre data (e.g. slip angle vs lateral force)
            tyre_data_long: DataFrame with longitudinal tyre data (e.g. slip ratio vs longitudinal force)
        """
        self.tyre_data_lat = tyre_data_lat
        self.tyre_data_long = tyre_data_long
        self.lat_force_interp = interp1d(
            tyre_data_lat['Slip Angle [deg]'],
            tyre_data_lat['Lateral Force [N]'],
            kind='linear',
            fill_value='extrapolate'
        )
        self.tyre_data = self.setup_from_data(tyre_data_lat, tyre_data_long)
        if self.tyre_data is None:
            raise ValueError("tyre_data must be a pandas DataFrame and cannot be None.")
        
    def setup_from_data(self, df_lat: pd.DataFrame, df_long: pd.DataFrame):
        """Setup tyre model from DataFrames."""
        return {
            'lat_tyre_data': df_lat,
            'long_tyre_data': df_long
        }

    def get_lateral_force(self, slip_angle: float, normal_load: float = None) -> float:
        """
        Get lateral force for a given slip angle.
        For MVP, we ignore normal_load (use constant from dummy data).
        
        Args:
            slip_angle: Slip angle in degrees
        Returns:
            Lateral force in N
        """
        return float(self.lat_force_interp(slip_angle))
    
class Vehicle:
    """Complete vehicle model combining all subsystems."""
    
    def __init__(self, parameters: vehicle_parameters, 
                 power_unit: PowerUnit, 
                 tyre_model: tyre_model,
                 config):
        """
        Initialize complete vehicle model.
        
        Args:
            parameters: Vehicle physical parameters
            power_unit: Power unit model
            tyre_model: Tyre model
            config: Configuration dictionary
        """
        self.params = parameters
        self.power_unit = power_unit
        self.tyre_model = tyre_model
        self.config = config
        
        # Calculate derived parameters
        self.weight = parameters.mass * 9.81  # N
        
    def compute_static_normal_load(self) -> float:
        """
        Compute static normal load per tire (assuming equal distribution).
        For point mass model without load transfer.
        
        Returns:
            Normal load per tire in N
        """
        return self.params.mass * 9.81 / 4.0
        
    def compute_tyre_forces(self, slip_angle_front: float, slip_angle_rear: float, g_lat: float = 0.0):
        """
        Compute lateral tire forces using the tire model, accounting for lateral load transfer.
        Args:
            slip_angle_front: Front tire slip angle in degrees
            slip_angle_rear: Rear tire slip angle in degrees
            g_lat: Lateral acceleration in m/s^2 (used for load transfer)
        Returns:
            (f_front, f_rear): Lateral forces in N
        """
        # Physical constants
        g = 9.81
        mass = self.params.mass
        weight = mass * g
        
        # Weight distribution
        # cog_longitudinal_pos is fraction of wheelbase from front axle
        cog_pos = self.params.cog_longitudinal_pos
        w_front = weight * (1.0 - cog_pos)
        w_rear = weight * cog_pos
        
        # Static wheel loads (per side)
        fz_fl_static = w_front / 2.0
        fz_fr_static = w_front / 2.0
        fz_rl_static = w_rear / 2.0
        fz_rr_static = w_rear / 2.0
        
        # Lateral Load Transfer
        # Total Load Transfer = Mass * ay * h_cg / Track_Width
        # We distribute load transfer based on static weight distribution (simplified LLTD)
        
        ay = abs(g_lat) # lateral acceleration magnitude
        h_cg = self.params.cog_z # height of center of gravity
        t_f = self.params.front_track_width # front track width
        t_r = self.params.rear_track_width # rear track width
        
        # Load transfer per axle (Total load transferred from inside to outside)
        delta_fz_f = (mass * ay * h_cg / t_f) * (w_front / weight)
        delta_fz_r = (mass * ay * h_cg / t_r) * (w_rear / weight)
        
        # Apply load transfer (Outside +, Inside -)
        # We calculate force for both sides and sum them
        fz_fl = max(0.0, fz_fl_static - delta_fz_f)
        fz_fr = max(0.0, fz_fr_static + delta_fz_f)
        
        fz_rl = max(0.0, fz_rl_static - delta_fz_r)
        fz_rr = max(0.0, fz_rr_static + delta_fz_r)
        
        # Compute lateral forces
        # Front
        fy_fl = self.tyre_model.get_lateral_force(slip_angle_front, normal_load=fz_fl)
        fy_fr = self.tyre_model.get_lateral_force(slip_angle_front, normal_load=fz_fr)
        f_front = fy_fl + fy_fr
        
        # Rear
        fy_rl = self.tyre_model.get_lateral_force(slip_angle_rear, normal_load=fz_rl)
        fy_rr = self.tyre_model.get_lateral_force(slip_angle_rear, normal_load=fz_rr)
        f_rear = fy_rl + fy_rr
        
        return f_front, f_rear
    
    def compute_yaw_moment(self, f_front: float, f_rear: float, steer_angle: float) -> float:
        """
        Compute yaw moment based on tire forces and steering angle.
        
        For steady-state cornering, yaw moment = 0 when:
        f_front * l_f = f_rear * l_r
        
        Args:
            f_front: Front lateral force in N
            f_rear: Rear lateral force in N
            steer_angle: Steering angle in degrees
        
        Returns:
            Yaw moment in Nm
        """
        # Calculate distances from CoG to axles
        l_f = self.params.wheelbase * (1 - self.params.cog_longitudinal_pos)  # distance to front
        l_r = self.params.wheelbase * self.params.cog_longitudinal_pos        # distance to rear
        # Yaw moment = (front force × front moment arm) - (rear force × rear moment arm)
        # In steady state, this should equal zero
        m_z = f_front * l_f - f_rear * l_r
        return m_z
    
    def compute_lateral_acceleration(self, f_front: float, f_rear: float, v_car: float) -> float:
        """
        Compute lateral acceleration based on tire forces and vehicle speed.
        Args:
            f_front: Front lateral force in N
            f_rear: Rear lateral force in N
            v_car: Vehicle speed in m/s
        Returns:
            Lateral acceleration in m/s^2
        """
        a_y = (f_front + f_rear) / self.params.mass
        return a_y
    
    def compute_aero_drag(self, v_car: float) -> float:
        """
        Compute aerodynamic drag force at given speed.
        
        F_drag = 0.5 * rho * Cd * A * v^2
        
        Args:
            v_car: Vehicle speed in m/s
            
        Returns:
            Drag force in N
        """
        # Use air density from config if available, else default to 1.225
        rho = self.config.get('ambient_conditions', {}).get('air_density', 1.225)
        f_drag = 0.5 * rho * self.params.drag_coefficient * self.params.frontal_area * v_car**2
        return f_drag
    
    def compute_downforce(self, v_car: float) -> float:
        """
        Compute aerodynamic downforce at given speed.
        
        F_downforce = 0.5 * rho * Cl * A * v^2
        
        Args:
            v_car: Vehicle speed in m/s
            
        Returns:
            Downforce in N
        """
        # Use air density from config if available, else default to 1.225
        rho = self.config.get('ambient_conditions', {}).get('air_density', 1.225)
        f_downforce = 0.5 * rho * self.params.downforce_coefficient * self.params.frontal_area * v_car**2
        return f_downforce
    
    def speed_to_rpm(self, v_car: float, gear_ratio: float) -> float:
        """
        Convert vehicle speed to engine RPM for given gear ratio.
        
        RPM = (v * 60 * gearRatio * finalDrive) / (2 * pi * wheel_radius)
        
        Args:
            v_car: Vehicle speed in m/s
            gear_ratio: Gear ratio (dimensionless)
            
        Returns:
            Engine RPM
        """
        if v_car < 0.01:  # Avoid division issues at very low speeds
            return 0.0
        
        # RPM = (speed * 60 * total_gear_ratio) / (2 * pi * wheel_radius)
        total_gear_ratio = gear_ratio * self.params.final_drive_ratio
        rpm = (v_car * 60 * total_gear_ratio) / (2 * np.pi * self.params.wheel_radius)
        return rpm
    
    def compute_wheel_torque(self, rpm: float, gear_ratio: float, throttle: float = 1.0) -> float:
        """
        Compute wheel torque from engine RPM and gear ratio.
        
        T_wheel = T_engine * gear_ratio * final_drive * efficiency
        
        Args:
            rpm: Engine RPM
            gear_ratio: Gear ratio (dimensionless)
            throttle: Throttle position (0 to 1)
            
        Returns:
            Wheel torque in Nm
        """
        # Get engine torque from powertrain
        engine_torque = self.power_unit.get_torque(rpm, throttle)
        # Apply gear ratios and transmission efficiency
        total_gear_ratio = gear_ratio * self.params.final_drive_ratio
        wheel_torque = engine_torque * total_gear_ratio * self.params.transmission_efficiency
        return wheel_torque
    
    def compute_longitudinal_force(self, v_car: float, gear_ratio: float, throttle: float = 1.0) -> float:
        """
        Compute longitudinal force at wheels for given speed and gear.
        
        F_x = T_wheel / wheel_radius - F_drag
        
        Args:
            v_car: Vehicle speed in m/s
            gear_ratio: Gear ratio (dimensionless)
            throttle: Throttle position (0 to 1)
            
        Returns:
            Net longitudinal force in N
        """
        # Convert speed to RPM
        rpm = self.speed_to_rpm(v_car, gear_ratio)
        # Get wheel torque
        wheel_torque = self.compute_wheel_torque(rpm, gear_ratio, throttle)
        # Convert torque to force at wheel contact patch
        f_traction = wheel_torque / self.params.wheel_radius
        # Subtract drag force
        f_drag = self.compute_aero_drag(v_car)
        # Net longitudinal force
        f_x = f_traction - f_drag
        return f_x
    
    def select_optimal_gear(self, v_car: float) -> float:
        """
        Select the optimal gear for maximum acceleration at given speed.
        
        Args:
            v_car: Vehicle speed in m/s
            
        Returns:
            Optimal gear ratio
        """
        max_force = -float('inf')
        optimal_gear = self.params.gear_ratios[0]
        
        # Try each gear and find which gives maximum force
        for gear_ratio in self.params.gear_ratios:
            f_x = self.compute_longitudinal_force(v_car, gear_ratio)
            if f_x > max_force:
                max_force = f_x
                optimal_gear = gear_ratio
        return optimal_gear
        
from vehicle.Tyres.baseTyre import create_tyre_model
from .Powertrain.basePowertrain import create_powertrain_model
import json
import os
def load_vehicle_parameters(file_path: str) -> vehicle_parameters:
    """
    Load vehicle parameters from a JSON configuration file.
    Args:
        file_path: Path to the parameters JSON file
    Returns:
        vehicle_parameters object
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Vehicle parameters file not found: {file_path}")
    with open(file_path, 'r') as f:
        data = json.load(f)
    # Read parameters from JSON structure
    params = vehicle_parameters(
        name=data['general']['name'],
        mass=data['general']['mass'],
        base_mu=data['general']['base_mu'],
        frontal_area=data['aerodynamics']['frontal_area'],
        drag_coefficient=data['aerodynamics']['drag_coefficient'],
        downforce_coefficient=data['aerodynamics']['downforce_coefficient'],
        aero_centre_of_pressure=data['aerodynamics']['aero_cp'],
        wheelbase=data['geometry']['wheelbase'],
        front_track_width=data['geometry']['front_track_width'],
        rear_track_width=data['geometry']['rear_track_width'],
        cog_z=data['geometry']['cog_z'],
        cog_longitudinal_pos=data['geometry']['cog_longitudinal_pos'],
        max_cog_z=data['geometry']['max_cog_z'],
        wheel_radius=data['drivetrain']['wheel_radius'],
        final_drive_ratio=data['drivetrain']['final_drive_ratio'],
        gear_ratios=data['drivetrain']['gear_ratios'],
        transmission_efficiency=data['drivetrain']['transmission_efficiency'],
        roll_stiffness=data['vehicle_dynamics']['roll_stiffness'],
        suspension_stiffness=data['vehicle_dynamics']['suspension_stiffness'],
        damping_coefficient=data['vehicle_dynamics']['damping_coefficient'],
        max_roll_angle_deg=data['vehicle_dynamics']['max_roll_angle_deg']
    )
    logger.info(f"Loaded vehicle parameters from {file_path}")
    return params

def create_vehicle(config) -> Vehicle:
    """Create vehicle from configuration file."""
    # Get vehicle parameters file_path from config
    params_filepath = config.get('vehicle_parameters', 'datasets/vehicle/parameters.json')
    # Load parameters from file
    params = load_vehicle_parameters(params_filepath)
    power_unit = create_powertrain_model(config.get('powertrain', {}))
    tyre_model = create_tyre_model(config.get('tyre_model', {}))
    loaded_vehicle = Vehicle(params, power_unit, tyre_model, config)
    return loaded_vehicle