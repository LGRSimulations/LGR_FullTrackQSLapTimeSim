import logging
import numpy as np
from dataclasses import dataclass
from typing import Optional
import pandas as pd
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
    trackWidth: float  # m
    coGHeight: float  # m
    maxGLat: float  # g (tire grip limit for point mass)
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
    
class TireModel:
    """
    Tire model for grip calculations.
    Use interpolated lookup tables for tyre forces.
    """
    
    def __init__(self, tireDataLat: pd.DataFrame, tireDataLong: pd.DataFrame):
        """
        Initialize tire model.
        
        Args:
            tireDataLat: DataFrame with lateral tire data (e.g. slip angle vs lateral force)
            tireDataLong: DataFrame with longitudinal tire data (e.g. slip ratio vs longitudinal force)
            """
        self.tireDataLat = tireDataLat
        self.tireDataLong = tireDataLong

        tireData = self.setupFromData(tireDataLat, tireDataLong)
        if tireData is None:
            raise ValueError("tireData must be a pandas DataFrame and cannot be None.")
    def setupFromData(self, dfLat: pd.DataFrame, dfLong: pd.DataFrame):
        """Setup tire model from DataFrames."""
        combinedTireData = {
            'latTireData': dfLat,
            'longTireData': dfLong
        }

        return combinedTireData

class Vehicle:
    """Complete vehicle model combining all subsystems."""
    
    def __init__(self, parameters: VehicleParameters, 
                 powerUnit: PowerUnit, 
                 tireModel: TireModel):
        """
        Initialize complete vehicle model.
        
        Args:
            parameters: Vehicle physical parameters
            powerUnit: Power unit model
            tireModel: Tire model
        """
        self.params = parameters
        self.power_unit = powerUnit
        self.tire_model = tireModel
        
        # Calculate derived parameters
        self.weight = parameters.mass * 9.81  # N
        

def createVehicle() -> Vehicle:
    """Create a default vehicle with dummy parameters."""
    params = VehicleParameters(
        name="TestCar",
        mass=1500.0,
        frontalArea=2.2,
        dragCoefficient=0.32,
        downforceCoefficient=2.5,
        aeroCentreOfPressure=1.0,
        wheelbase=2.5,
        trackWidth=1.6,
        coGHeight=0.55,
        maxGLat=1.2,
        maxGLongAccel=0.8,
        maxGLongBrake=1.0
    )
    # Create dummy powerData DataFrame
    rpm = np.array([0, 1000, 2000, 3000, 4000, 5000])           # RPM at crankshaft   
    torque = np.array([600, 600, 550, 500, 400, 300])           # Torque at crankshaft (Nm)      
    power = (rpm * torque * 2 * np.pi) / 60                     # Power at crankshaft (W)
    efficiency = np.array([85, 88, 90, 92, 90, 88])             # Powertrain efficiency (%)
    fuelConsumption = np.array([250, 220, 200, 190, 210, 230])  # g/kWh
    gear = np.array([1, 2, 3, 4, 5, 6])                         # Gear number
    throttle = np.array([100, 100, 100, 100, 100, 100])         # Throttle position (%)
    finalDrive = np.array([3.5, 3.5, 3.5, 3.5, 3.5, 3.5])       # Final drive ratio

    powerData = pd.DataFrame({
        'Motor Speed [RPM]': rpm,
        'Torque [Nm]': torque,
        'Continuous Power [kW]': power / 1000,
        'Peak Power [kW]': power / 1000,
        'Efficiency [%]': efficiency,
        'Fuel Consumption [g/kWh]': fuelConsumption,
        'Gear': gear,
        'Throttle Position [%]': throttle,
        'Final Drive Ratio': finalDrive,
        'Max RPM': [5000]*6,
        'Min RPM': [0]*6
    })
    # Create dummy tire data DataFrames
    # Lateral tire data (cornering)
    tireDataLat = pd.DataFrame({
        'Slip Angle [deg]': [-15, -10, -5, 0, 5, 10, 15],
        'Lateral Force [N]': [-8000, -6000, -3000, 0, 3000, 6000, 8000],
        'Normal Load [N]': [3000]*7,           # constant for now
        'Camber Angle [deg]': [0]*7,           # neutral camber
        'Tire Pressure [kPa]': [200]*7,        # constant for now
        'Temperature [C]': [25]*7              # constant for now
    })

    # Longitudinal tire data (traction/braking)
    tireDataLong = pd.DataFrame({
        'Slip Ratio [%]': [-0.2, -0.1, -0.05, 0, 0.05, 0.1, 0.2],
        'Longitudinal Force [N]': [-4000, -2000, -1000, 0, 1000, 2000, 4000],
        'Normal Load [N]': [3000]*7,
        'Tire Pressure [kPa]': [200]*7,
        'Temperature [C]': [25]*7
    })


    powerUnit = PowerUnit(powerData)
    tireModel = TireModel(tireDataLat, tireDataLong)
    loadedVehicle = Vehicle(params, powerUnit, tireModel)
    return loadedVehicle