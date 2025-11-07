import pandas as pd
import numpy as np
import os
from scipy.interpolate import interp1d
from abc import ABC, abstractmethod
from typing import Dict, Any

class BasePowertrainModel(ABC):
    """
    Abstract base class for powertrain models.
    All powertrain models should inherit from this class and implement its abstract methods.
    """
    
    def __init__(self):
        """Initialize the base powertrain model."""
        pass
    
    @classmethod
    @abstractmethod
    def from_config(cls, config: Dict[str, Any]):
        """
        Create a powertrain model from configuration.
        
        Args:
            config: Dictionary containing configuration parameters
            
        Returns:
            An instance of the powertrain model
        """
        pass
    
    @abstractmethod
    def getTorque(self, rpm: float, throttle: float = 1.0) -> float:
        """
        Get torque for given RPM and throttle position.
        
        Args:
            rpm: Engine/motor speed in RPM
            throttle: Throttle position (0 to 1)
            
        Returns:
            Torque in Nm
        """
        pass
    
    @abstractmethod
    def getPower(self, rpm: float, throttle: float = 1.0) -> float:
        """
        Get power for given RPM and throttle position.
        
        Args:
            rpm: Engine/motor speed in RPM
            throttle: Throttle position (0 to 1)
            
        Returns:
            Power in kW
        """
        pass
    
    @abstractmethod
    def getEfficiency(self, rpm: float, throttle: float = 1.0) -> float:
        """
        Get efficiency for given RPM and throttle position.
        
        Args:
            rpm: Engine/motor speed in RPM
            throttle: Throttle position (0 to 1)
            
        Returns:
            Efficiency as a percentage (0-100)
        """
        pass


class LookupTablePowertrainModel(BasePowertrainModel):
    """
    Powertrain model that uses lookup tables for torque, power, and efficiency calculations.
    """
    
    def __init__(self, powertrainData: pd.DataFrame):
        """
        Initialize lookup table powertrain model.
        
        Args:
            powertrainData: DataFrame with powertrain data including RPM, torque, power, and efficiency
        """
        super().__init__()
        self.powertrainData = powertrainData
        
        # Note: We don't create a torque interpolator because torque will be calculated from power
        # self.torqueInterp = self._create_interpolator('Torque [Nm]')
        
        # Create interpolation functions for power (use Continuous Power, fallback to Peak Power)
        if 'Continuous Power [kW]' in powertrainData.columns:
            self.powerInterp = self._create_interpolator('Continuous Power [kW]')
        elif 'Peak Power [kW]' in powertrainData.columns:
            self.powerInterp = self._create_interpolator('Peak Power [kW]')
        else:
            raise ValueError("Powertrain data must contain either 'Continuous Power [kW]' or 'Peak Power [kW]' column")
        
        # Create interpolation functions for efficiency (if available)
        self.efficiencyInterp = self._create_interpolator('Efficiency [%]')
        
        # Store min/max RPM
        self.maxRPM = powertrainData['Max RPM'].max() if 'Max RPM' in powertrainData.columns else 5000
        self.minRPM = powertrainData['Min RPM'].min() if 'Min RPM' in powertrainData.columns else 0
    
    def _create_interpolator(self, column_name):
        """Create an interpolation function for the given column."""
        if column_name in self.powertrainData.columns:
            return interp1d(  # Fix: use interp1d from scipy.interpolate
                self.powertrainData['Motor Speed [RPM]'],
                self.powertrainData[column_name],
                kind='linear',
                bounds_error=False,
                fill_value=(self.powertrainData[column_name].iloc[0], self.powertrainData[column_name].iloc[-1])
            )
        else:
            # Return a default interpolator that gives zeros
            # Fix: Use a lambda function instead of numpy's interp1d
            return lambda x: 0
    
    @classmethod
    def from_config(cls, config: Dict[str, Any]):
        """
        Create a lookup table powertrain model from configuration.
        
        Args:
            config: Dictionary containing configuration parameters with filepath to powertrain data
            
        Returns:
            An instance of LookupTablePowertrainModel
        """
        filepath = config.get('powertrain')  # Note: using 'powertrain' key to match your config format
        if not filepath:
            raise ValueError("Powertrain model config must specify a 'powertrain' filepath")
        
        # Get absolute path relative to project root
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        full_path = os.path.join(project_root, filepath)
        
        # Read and process the powertrain data file
        try:
            # Check if file exists
            if not os.path.exists(full_path):
                raise FileNotFoundError(f"Powertrain data file not found: {full_path}")
                
            # Read the powertrain data
            if filepath.endswith('.csv'):
                df = pd.read_csv(full_path)
                
                # Ensure required columns exist
                required_columns = ['Motor Speed [RPM]']
                for column in required_columns:
                    if column not in df.columns:
                        raise ValueError(f"Required column '{column}' not found in powertrain data")
                
                # Return the model
                return cls(df)
                
            else:
                raise ValueError(f"Unsupported file format: {filepath}")
                
        except Exception as e:
            import logging
            logging.error(f"Failed to load powertrain data from {filepath}: {str(e)}")
            raise
    
    def getTorque(self, rpm: float, throttle: float = 1.0) -> float:
        """
        Get torque for given RPM and throttle position.
        Calculates torque from power: T = P / (2π × RPM / 60) = (P × 60) / (2π × RPM)
        
        Args:
            rpm: Engine/motor speed in RPM
            throttle: Throttle position (0 to 1)
            
        Returns:
            Torque in Nm
        """
        # Handle zero or very low RPM
        if rpm < 1.0:
            # At very low RPM, return a reasonable starting torque
            # Use power at minimum RPM to estimate
            power_kW = self.getPower(1.0, throttle)
            # Assume a reasonable torque at standstill (e.g., power at min RPM converted)
            return (power_kW * 1000 * 60) / (2 * np.pi * 1.0) if power_kW > 0 else 0.0
        
        # Ensure RPM is within bounds
        rpm = max(self.minRPM, min(rpm, self.maxRPM))
        rpm = float(rpm)  # Ensure rpm is a native Python float
        
        # Get power at this RPM
        power_kW = self.getPower(rpm, throttle)
        
        # Convert power (kW) to torque (Nm)
        # P (Watts) = T (Nm) × ω (rad/s)
        # ω (rad/s) = 2π × RPM / 60
        # T = P / ω = P × 60 / (2π × RPM)
        power_watts = power_kW * 1000
        torque = (power_watts * 60) / (2 * np.pi * rpm)
        
        return torque
    
    def getPower(self, rpm: float, throttle: float = 1.0) -> float:
        """
        Get power for given RPM and throttle position.
        
        Args:
            rpm: Engine/motor speed in RPM
            throttle: Throttle position (0 to 1)
            
        Returns:
            Power in kW
        """
        # Ensure RPM is within bounds
        rpm = max(self.minRPM, min(rpm, self.maxRPM))
        rpm = float(rpm)
        
        # Interpolate power from the curve
        # Use continuous power curve (or peak if continuous not available)
        base_power = float(self.powerInterp(rpm))
        
        return base_power * throttle
    
    def getEfficiency(self, rpm: float, throttle: float = 1.0) -> float:
        """
        Get efficiency for given RPM and throttle position.
        
        Args:
            rpm: Engine/motor speed in RPM
            throttle: Throttle position (0 to 1)
            
        Returns:
            Efficiency as a percentage (0-100)
        """
        # Ensure RPM is within bounds
        rpm = max(self.minRPM, min(rpm, self.maxRPM))
        
        # Get efficiency
        efficiency = float(self.efficiencyInterp(rpm))
        
        # For more advanced models, efficiency might vary with throttle position
        return efficiency


# Factory function to create appropriate powertrain model based on config
def createPowertrainModel(config: Dict[str, Any]) -> BasePowertrainModel:
    """
    Factory function to create a powertrain model based on configuration.
    
    Args:
        config: Dictionary containing configuration parameters
        
    Returns:
        An instance of a BasePowertrainModel subclass
    """
    model_type = config.get('type', 'lookup')
    
    if model_type.lower() == 'lookup' or model_type.lower() == 'lookuptable':
        return LookupTablePowertrainModel.from_config(config)
    # TODO: Implement support for custom powertrain models here.
    else:
        raise ValueError(f"Unknown powertrain model type: {model_type}")