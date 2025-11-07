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
    wheelRadius: float  # m (effective rolling radius)
    finalDriveRatio: float  # dimensionless (final drive gear ratio)
    gearRatios: list  # dimensionless (gear ratios for each gear)
    transmissionEfficiency: float  # dimensionless (0-1, mechanical efficiency)

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
    
    def computeAeroDrag(self, vCar: float) -> float:
        """
        Compute aerodynamic drag force at given speed.
        
        F_drag = 0.5 * rho * Cd * A * v^2
        
        Args:
            vCar: Vehicle speed in m/s
            
        Returns:
            Drag force in N
        """
        rho = 1.225  # Air density in kg/m^3 at sea level
        F_drag = 0.5 * rho * self.params.dragCoefficient * self.params.frontalArea * vCar**2
        return F_drag
    
    def computeDownforce(self, vCar: float) -> float:
        """
        Compute aerodynamic downforce at given speed.
        
        F_downforce = 0.5 * rho * Cl * A * v^2
        
        Args:
            vCar: Vehicle speed in m/s
            
        Returns:
            Downforce in N
        """
        rho = 1.225  # Air density in kg/m^3 at sea level
        F_downforce = 0.5 * rho * self.params.downforceCoefficient * self.params.frontalArea * vCar**2
        return F_downforce
    
    def speedToRPM(self, vCar: float, gearRatio: float) -> float:
        """
        Convert vehicle speed to engine RPM for given gear ratio.
        
        RPM = (v * 60 * gearRatio * finalDrive) / (2 * pi * wheelRadius)
        
        Args:
            vCar: Vehicle speed in m/s
            gearRatio: Gear ratio (dimensionless)
            
        Returns:
            Engine RPM
        """
        if vCar < 0.01:  # Avoid division issues at very low speeds
            return 0.0
        
        # RPM = (speed * 60 * total_gear_ratio) / (2 * pi * wheel_radius)
        totalGearRatio = gearRatio * self.params.finalDriveRatio
        rpm = (vCar * 60 * totalGearRatio) / (2 * np.pi * self.params.wheelRadius)
        return rpm
    
    def computeWheelTorque(self, rpm: float, gearRatio: float, throttle: float = 1.0) -> float:
        """
        Compute wheel torque from engine RPM and gear ratio.
        
        T_wheel = T_engine * gearRatio * finalDrive * efficiency
        
        Args:
            rpm: Engine RPM
            gearRatio: Gear ratio (dimensionless)
            throttle: Throttle position (0 to 1)
            
        Returns:
            Wheel torque in Nm
        """
        # Get engine torque from powertrain
        engineTorque = self.powerUnit.getTorque(rpm, throttle)
        
        # Apply gear ratios and transmission efficiency
        totalGearRatio = gearRatio * self.params.finalDriveRatio
        wheelTorque = engineTorque * totalGearRatio * self.params.transmissionEfficiency
        
        return wheelTorque
    
    def computeLongitudinalForce(self, vCar: float, gearRatio: float, throttle: float = 1.0) -> float:
        """
        Compute longitudinal force at wheels for given speed and gear.
        
        F_x = T_wheel / wheelRadius - F_drag
        
        Args:
            vCar: Vehicle speed in m/s
            gearRatio: Gear ratio (dimensionless)
            throttle: Throttle position (0 to 1)
            
        Returns:
            Net longitudinal force in N
        """
        # Convert speed to RPM
        rpm = self.speedToRPM(vCar, gearRatio)
        
        # Get wheel torque
        wheelTorque = self.computeWheelTorque(rpm, gearRatio, throttle)
        
        # Convert torque to force at wheel contact patch
        F_traction = wheelTorque / self.params.wheelRadius
        
        # Subtract drag force
        F_drag = self.computeAeroDrag(vCar)
        
        # Net longitudinal force
        F_x = F_traction - F_drag
        
        return F_x
    
    def selectOptimalGear(self, vCar: float) -> float:
        """
        Select the optimal gear for maximum acceleration at given speed.
        
        Args:
            vCar: Vehicle speed in m/s
            
        Returns:
            Optimal gear ratio
        """
        maxForce = -float('inf')
        optimalGear = self.params.gearRatios[0]
        
        # Try each gear and find which gives maximum force
        for gearRatio in self.params.gearRatios:
            F_x = self.computeLongitudinalForce(vCar, gearRatio)
            if F_x > maxForce:
                maxForce = F_x
                optimalGear = gearRatio
        
        return optimalGear
        
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
        wheelRadius=data['drivetrain']['wheelRadius'],
        finalDriveRatio=data['drivetrain']['finalDriveRatio'],
        gearRatios=data['drivetrain']['gearRatios'],
        transmissionEfficiency=data['drivetrain']['transmissionEfficiency']
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