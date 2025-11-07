import logging
import numpy as np
from dataclasses import dataclass
import pandas as pd
from scipy.interpolate import interp1d

logger = logging.getLogger(__name__)

@dataclass
class VehicleParameters:
    """Core vehicle parameters for point mass simulation."""
    name: str 
    mass: float  # kg
    frontalArea: float  # m²
    dragCoefficient: float  # dimensionless
    downforceCoefficient: float  # dimensionless (L/D ratio typically)
    aeroCentreOfPressure: float  # m from front axle
    wheelbase: float  # m
    frontTrackWidth: float  # m
    rearTrackWidth: float  # m
    coGHeight: float  # m
    coGLongitudinalPos: float  # fraction of wheelbase from front axle (0 to 1)
    maxGLat: float  # g (tyre grip limit for point mass)
    maxGLongAccel: float  # g (acceleration limit)
    maxGLongBrake: float  # g (braking limit)

class PowerUnit:
    """
    Power unit model with torque/power curves and efficiency maps.
    Handles both ICE and electric powertrains.
    """
    
    def __init__(self, powerData: pd.DataFrame):
        """
        Initialize power unit.
        
        Args:
            powerData: DataFrame with columns ['Motor Speed [RPM]', 'Continuous Power [kW]', 'Peak Power [kW]']
            maxPower: Maximum power in Watts
            maxTorque: Maximum torque in Nm
            maxRPM: Maximum RPM
        """

        powerData = self.setupFromData(powerData)
        if powerData is None:
            raise ValueError("powerData must be a pandas DataFrame and cannot be None.")
        
    def setupFromData(self, df: pd.DataFrame):
        """Setup power unit from DataFrame."""
        powerData = df
        return powerData
    
class TyreModel:
    """
    tyre model for grip calculations.
    Use interpolated lookup tables for tyre forces.
    """
    
    def __init__(self, tyreDataLat: pd.DataFrame, tyreDataLong: pd.DataFrame):
        """
        Initialize tyre model.
        
        Args:
            tyreDataLat: DataFrame with lateral tyre data (e.g. slip angle vs lateral force)
            tyreDataLong: DataFrame with longitudinal tyre data (e.g. slip ratio vs longitudinal force)
            """
        self.tyreDataLat = tyreDataLat
        self.tyreDataLong = tyreDataLong

        # Create interpolation functions for lateral forces
        self.latForceInterp = interp1d(
            tyreDataLat['Slip Angle [deg]'],
            tyreDataLat['Lateral Force [N]'],
            kind='linear',
            fill_value='extrapolate'
        )

        tyreData = self.setupFromData(tyreDataLat, tyreDataLong)
        if tyreData is None:
            raise ValueError("tyreData must be a pandas DataFrame and cannot be None.")
        
    def setupFromData(self, dfLat: pd.DataFrame, dfLong: pd.DataFrame):
        """Setup tyre model from DataFrames."""
        combinedtyreData = {
            'latTyreData': dfLat,
            'longTyreData': dfLong
        }

        return combinedtyreData

    def getLateralForce(self, slipAngle: float) -> float:
        """
        Get lateral force for a given slip angle.
        For MVP, we ignore normal_load (use constant from dummy data).
        
        Args:
            slipAngle: Slip angle in degrees
            normalLoad: Normal load in N (ignored for now)
            
        Returns:
            Lateral force in N
        """
        return float(self.latForceInterp(slipAngle))
    
class Vehicle:
    """Complete vehicle model combining all subsystems."""
    
    def __init__(self, parameters: VehicleParameters, 
                 powerUnit: PowerUnit, 
                 tyreModel: TyreModel,
                 config):
        """
        Initialize complete vehicle model.
        
        Args:
            parameters: Vehicle physical parameters
            powerUnit: Power unit model
            tyreModel: Tyre model
            config: Configuration dictionary
        """
        self.params = parameters
        self.powerUnit = powerUnit
        self.tyreModel = tyreModel
        self.config = config
        
        # Calculate derived parameters
        self.weight = parameters.mass * 9.81  # N
        
    def computeStaticNormalLoad(self) -> float:
        """
        Compute static normal load per tire (assuming equal distribution).
        For point mass model without load transfer.
        
        Returns:
            Normal load per tire in N
        """
        return self.params.mass * 9.81 / 4.0
        
    def computeTyreForces(self, slipAngleFront: float, slipAngleRear: float):
        """
        Compute lateral tire forces using the tire model.
        
        Args:
            slip_angle_front: Front tire slip angle in degrees
            slip_angle_rear: Rear tire slip angle in degrees
            
        Returns:
            (F_front, F_rear): Lateral forces in N
        """
        # Calculate static normal load per tire
        tyreNormalLoad = self.computeStaticNormalLoad()
        # Get lateral forces from tyre model
        # For now, assume both front and rear tires have same normal load
        F_front = self.tyreModel.getLateralForce(slipAngleFront, normalLoad=tyreNormalLoad)*2  # two front tires
        F_rear = self.tyreModel.getLateralForce(slipAngleRear, normalLoad=tyreNormalLoad)*2    # two rear tires
        return F_front, F_rear
    
    def computeYawMoment(self, F_front: float, F_rear: float, aSteer: float) -> float:
        """
        Compute yaw moment based on tire forces and steering angle.
        
        For steady-state cornering, yaw moment = 0 when:
        F_front * l_f = F_rear * l_r
        
        Args:
            F_front: Front lateral force in N
            F_rear: Rear lateral force in N
            aSteer: Steering angle in degrees
        
        Returns:
            Yaw moment in Nm
        """
        # Calculate distances from CoG to axles
        l_f = self.params.wheelbase * (1 - self.params.coGLongitudinalPos)  # distance to front
        l_r = self.params.wheelbase * self.params.coGLongitudinalPos        # distance to rear
        
        # Yaw moment = (front force × front moment arm) - (rear force × rear moment arm)
        # In steady state, this should equal zero
        M_z = F_front * l_f - F_rear * l_r
        
        return M_z
    
    def computeLateralAcceleration(self, F_front: float, F_rear: float, vCar: float) -> float:
        """
        Compute lateral acceleration based on tire forces.
        
        Args:
            F_front: Front lateral force in N
            F_rear: Rear lateral force in N
            vCar: Vehicle speed in m/s (not used in point mass model)
            
        Returns:
            Lateral acceleration in m/s²
        """
        a_y = (F_front + F_rear) / self.params.mass
        return a_y
        
from .Tyres.baseTyre import createTyreModel
from .Powertrain.basePowertrain import createPowertrainModel
import json
import os

def loadVehicleParameters(filepath: str) -> VehicleParameters:
    """
    Load vehicle parameters from a JSON configuration file.
    
    Args:
        filepath: Path to the parameters JSON file
        
    Returns:
        VehicleParameters object
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Vehicle parameters file not found: {filepath}")
    
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    # Read parameters from JSON structure
    params = VehicleParameters(
        name=data['general']['name'],
        mass=data['general']['mass'],
        frontalArea=data['aerodynamics']['frontalArea'],
        dragCoefficient=data['aerodynamics']['dragCoefficient'],
        downforceCoefficient=data['aerodynamics']['downforceCoefficient'],
        aeroCentreOfPressure=data['aerodynamics']['aeroCentreOfPressure'],
        wheelbase=data['geometry']['wheelbase'],
        frontTrackWidth=data['geometry']['frontTrackWidth'],
        rearTrackWidth=data['geometry']['rearTrackWidth'],
        coGHeight=data['geometry']['coGHeight'],
        coGLongitudinalPos=data['geometry']['coGLongitudinalPos'],
        maxGLat=data['performance']['maxGLat'],
        maxGLongAccel=data['performance']['maxGLongAccel'],
        maxGLongBrake=data['performance']['maxGLongBrake']
    )
    
    logger.info(f"Loaded vehicle parameters from {filepath}")
    return params

def createVehicle(config) -> Vehicle:
    """Create vehicle from configuration file."""
    # Get vehicle parameters filepath from config
    paramsFilepath = config.get('vehicleParameters', 'datasets/vehicle/parameters.json')
    
    # Load parameters from file
    params = loadVehicleParameters(paramsFilepath)
    
    powerUnit = createPowertrainModel(config.get('powertrain', {}))
    tyreModel = createTyreModel(config.get('tyreModel', {}))
    loadedVehicle = Vehicle(params, powerUnit, tyreModel, config)
    return loadedVehicle