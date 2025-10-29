import pandas as pd
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
        
        # Create interpolation functions for torque
        self.torqueInterp = self._create_interpolator('Torque [Nm]')
        
        # Create interpolation functions for power
        self.powerInterp = self._create_interpolator('Continuous Power [kW]')
        
        # Create interpolation functions for efficiency
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
        
        Args:
            rpm: Engine/motor speed in RPM
            throttle: Throttle position (0 to 1)
            
        Returns:
            Torque in Nm
        """
        # Ensure RPM is within bounds
        rpm = max(self.minRPM, min(rpm, self.maxRPM))
        
        # Get base torque and scale by throttle
        base_torque = float(self.torqueInterp(rpm))
        return base_torque * throttle
    
    def getPower(self, rpm: float, throttle: float = 1.0) -> float:
        """
        Get power for given RPM and throttle position.
        
        Args:
            rpm: Engine/motor speed in RPM
            throttle: Throttle position (0 to 1)
            
        Returns:
            Power in kW
        """
        # Option 1: Interpolate directly from power curve
        base_power = float(self.powerInterp(rpm))
        
        # Option 2: Calculate from torque
        # torque = self.getTorque(rpm, throttle)
        # power_watts = (rpm * torque * 2 * np.pi) / 60
        # base_power = power_watts / 1000  # convert to kW
        
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