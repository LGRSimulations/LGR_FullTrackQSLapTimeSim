import logging
import numpy as np
from dataclasses import dataclass
from typing import Optional
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
    trackWidth: float  # m
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
                 TyreModel: TyreModel):
        """
        Initialize complete vehicle model.
        
        Args:
            parameters: Vehicle physical parameters
            powerUnit: Power unit model
            TyreModel: tyre model
        """
        self.params = parameters
        self.power_unit = powerUnit
        self.tyre_model = TyreModel
        
        # Calculate derived parameters
        self.weight = parameters.mass * 9.81  # N
        
    def computeTyreForces(self, slipAngleFront: float, slipAngleRear: float):
        """
        Compute lateral tire forces using the tire model.
        
        Args:
            slip_angle_front: Front tire slip angle in degrees
            slip_angle_rear: Rear tire slip angle in degrees
            normal_load: Normal load on tire in N
            
        Returns:
            (F_front, F_rear): Lateral forces in N
        """
        F_front = self.tyre_model.getLateralForce(slipAngleFront)
        F_rear = self.tyre_model.getLateralForce(slipAngleRear)
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
        coGLongitudinalPos=0.5,
        maxGLat=1.2,
        maxGLongAccel=0.8,  # Ideally we get this from power unit
        maxGLongBrake=1.0   # Ideally we get this from brake system
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
    # Create dummy tyre data DataFrames
    # Lateral tyre data (cornering)
    tyreDataLat = pd.DataFrame({
        'Slip Angle [deg]': [-15, -10, -5, 0, 5, 10, 15],
        'Lateral Force [N]': [-8000, -6000, -3000, 0, 3000, 6000, 8000],
        'Normal Load [N]': [3000]*7,           # constant for now
        'Camber Angle [deg]': [0]*7,           # neutral camber
        'tyre Pressure [kPa]': [200]*7,        # constant for now
        'Temperature [C]': [25]*7              # constant for now
    })

    # Longitudinal tyre data (traction/braking)
    tyreDataLong = pd.DataFrame({
        'Slip Ratio [%]': [-0.2, -0.1, -0.05, 0, 0.05, 0.1, 0.2],
        'Longitudinal Force [N]': [-4000, -2000, -1000, 0, 1000, 2000, 4000],
        'Normal Load [N]': [3000]*7,
        'tyre Pressure [kPa]': [200]*7,
        'Temperature [C]': [25]*7
    })


    powerUnit = PowerUnit(powerData)
    tyreModel = TyreModel(tyreDataLat, tyreDataLong)
    loadedVehicle = Vehicle(params, powerUnit, tyreModel)
    return loadedVehicle