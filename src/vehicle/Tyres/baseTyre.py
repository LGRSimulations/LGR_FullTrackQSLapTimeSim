import pandas as pd
import numpy as np
import os
import json
from scipy.interpolate import interp1d
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional


class BaseTyreModel(ABC):
    """
    Abstract base class for tyre models.
    All tyre models should inherit from this class and implement its abstract methods.
    """
    
    def __init__(self):
        """Initialize the base tyre model."""
        pass
    
    @classmethod
    @abstractmethod
    def from_config(cls, config: Dict[str, Any]):
        """
        Create a tyre model from configuration.
        
        Args:
            config: Dictionary containing configuration parameters
            
        Returns:
            An instance of the tyre model
        """
        pass
    
    @abstractmethod
    def get_lateral_force(self, slipAngle: float, normalLoad: Optional[float] = None, 
                        camberAngle: Optional[float] = None, 
                        tyrePressure: Optional[float] = None,
                        temperature: Optional[float] = None) -> float:
        """
        Get lateral force for a given slip angle and conditions.
        
        Args:
            slipAngle: Slip angle in degrees
            normalLoad: Normal load in N (optional)
            camberAngle: Camber angle in degrees (optional)
            tyrePressure: Tyre pressure in kPa (optional)
            temperature: Tyre temperature in °C (optional)
            
        Returns:
            Lateral force in N
        """
        pass
    
    @abstractmethod
    def get_longitudinal_force(self, slipRatio: float, normalLoad: Optional[float] = None,
                            tyrePressure: Optional[float] = None,
                            temperature: Optional[float] = None) -> float:
        """
        Get longitudinal force for a given slip ratio and conditions.
        
        Args:
            slipRatio: Slip ratio (dimensionless)
            normalLoad: Normal load in N (optional)
            tyrePressure: Tyre pressure in kPa (optional)
            temperature: Tyre temperature in °C (optional)
            
        Returns:
            Longitudinal force in N
        """
        pass
    
    @abstractmethod
    def get_combined_forces(self, slipAngle: float, slipRatio: float, 
                         normalLoad: Optional[float] = None) -> tuple:
        """
        Get combined lateral and longitudinal forces when both slip angle and slip ratio exist.
        
        Args:
            slipAngle: Slip angle in degrees
            slipRatio: Slip ratio (dimensionless)
            normalLoad: Normal load in N (optional)
            
        Returns:
            Tuple of (lateral_force, longitudinal_force) in N
        """
        pass


class LookupTableTyreModel(BaseTyreModel):
    """
    Tyre model that uses lookup tables for force calculations.
    This is the current implementation converted to use the base class.
    """
    
    def __init__(self, tyre_data_lat: pd.DataFrame, tyre_data_long: pd.DataFrame):
        """
        Initialize lookup table tyre model.
        
        Args:
            tyre_data_lat: DataFrame with lateral tyre data
            tyre_data_long: DataFrame with longitudinal tyre data
        """
        super().__init__()
        self.tyre_data_lat = tyre_data_lat
        self.tyre_data_long = tyre_data_long
        self.lat_force_interp = interp1d(
            tyre_data_lat['Slip Angle [deg]'],
            tyre_data_lat['Lateral Force [N]'],
            kind='linear',
            fill_value='extrapolate'
        )
        self.long_force_interp = interp1d(
            tyre_data_long['Slip Ratio [%]'],
            tyre_data_long['Longitudinal Force [N]'],
            kind='linear',
            fill_value='extrapolate'
        )
    
    @classmethod
    def from_config(cls, config: Dict[str, Any]):
        """
        Create a lookup table tyre model from configuration.
        
        Args:
            config: Dictionary containing configuration parameters with file_path to tyre data
            
        Returns:
            An instance of LookupTableTyreModel
        """
        file_path = config.get('file_path')
        if not file_path:
            raise ValueError("Tyre model config must specify a file_path")
        
        # Get absolute path relative to project root
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        full_path = os.path.join(project_root, file_path)
        
        # Read and process the tyre data file
        # This assumes a specific CSV format - adapt as needed for your data format
        try:
            # Check if file exists
            if not os.path.exists(full_path):
                raise FileNotFoundError(f"Tyre data file not found: {full_path}")
                
            # Read the tyre data
            if file_path.endswith('.csv'):
                # For simple CSV format
                df = pd.read_csv(full_path)
                
                # Extract lateral and longitudinal data
                # This is a simplified example - adjust based on your actual CSV structure
                if 'dataType' in df.columns:
                    # If the CSV has a column identifying data type
                    tyre_data_lat = df[df['dataType'] == 'lateral'].reset_index(drop=True)
                    tyre_data_long = df[df['dataType'] == 'longitudinal'].reset_index(drop=True)
                else:
                    # If separate columns for lateral and longitudinal data
                    # Create lateral dataframe
                    tyre_data_lat = pd.DataFrame({
                        'Slip Angle [deg]': df['Slip Angle [deg]'] if 'Slip Angle [deg]' in df.columns else df['SlipAngle'],
                        'Lateral Force [N]': df['Lateral Force [N]'] if 'Lateral Force [N]' in df.columns else df['LatForce'],
                        'Normal Load [N]': df['Normal Load [N]'] if 'Normal Load [N]' in df.columns else [3000] * len(df),
                        'Camber Angle [deg]': df['Camber Angle [deg]'] if 'Camber Angle [deg]' in df.columns else [0] * len(df),
                        'Tyre Pressure [kPa]': df['Tyre Pressure [kPa]'] if 'Tyre Pressure [kPa]' in df.columns else [200] * len(df),
                        'Temperature [C]': df['Temperature [C]'] if 'Temperature [C]' in df.columns else [25] * len(df)
                    })
                    
                    # Create longitudinal dataframe
                    tyre_data_long = pd.DataFrame({
                        'Slip Ratio [%]': df['Slip Ratio [%]'] if 'Slip Ratio [%]' in df.columns else df['SlipRatio'],
                        'Longitudinal Force [N]': df['Longitudinal Force [N]'] if 'Longitudinal Force [N]' in df.columns else df['LongForce'],
                        'Normal Load [N]': df['Normal Load [N]'] if 'Normal Load [N]' in df.columns else [3000] * len(df),
                        'Tyre Pressure [kPa]': df['Tyre Pressure [kPa]'] if 'Tyre Pressure [kPa]' in df.columns else [200] * len(df),
                        'Temperature [C]': df['Temperature [C]'] if 'Temperature [C]' in df.columns else [25] * len(df)
                    })
            elif file_path.endswith('.json'):
                # For JSON format
                with open(full_path, 'r') as f:
                    tyre_data = json.load(f)
                
                # Extract data based on JSON structure
                lateral_data = tyre_data.get('lateral', {})
                longitudinal_data = tyre_data.get('longitudinal', {})
                
                # Create dataframes
                tyre_data_lat = pd.DataFrame({
                    'Slip Angle [deg]': lateral_data.get('slip_angles', []),
                    'Lateral Force [N]': lateral_data.get('forces', []),
                    'Normal Load [N]': lateral_data.get('normal_loads', [3000] * len(lateral_data.get('slip_angles', []))),
                    'Camber Angle [deg]': lateral_data.get('camber_angles', [0] * len(lateral_data.get('slip_angles', []))),
                    'Tyre Pressure [kPa]': lateral_data.get('pressures', [200] * len(lateral_data.get('slip_angles', []))),
                    'Temperature [C]': lateral_data.get('temperatures', [25] * len(lateral_data.get('slip_angles', [])))
                })
                
                tyre_data_long = pd.DataFrame({
                    'Slip Ratio [%]': longitudinal_data.get('slip_ratios', []),
                    'Longitudinal Force [N]': longitudinal_data.get('forces', []),
                    'Normal Load [N]': longitudinal_data.get('normal_loads', [3000] * len(longitudinal_data.get('slip_ratios', []))),
                    'Tyre Pressure [kPa]': longitudinal_data.get('pressures', [200] * len(longitudinal_data.get('slip_ratios', []))),
                    'Temperature [C]': longitudinal_data.get('temperatures', [25] * len(longitudinal_data.get('slip_ratios', [])))
                })
            else:
                raise ValueError(f"Unsupported file format: {file_path}")
                
            # Return the model
            return cls(tyre_data_lat, tyre_data_long)
            
        except Exception as e:
            import logging
            logging.error(f"Failed to load tyre data from {file_path}: {str(e)}")
            raise
    
    def get_lateral_force(self, slip_angle: float, normal_load: Optional[float] = None, 
                        camber_angle: Optional[float] = None, 
                        tyre_pressure: Optional[float] = None,
                        temperature: Optional[float] = None) -> float:
        """
        Get lateral force for a given slip angle.
        For MVP, we use simple interpolation and ignore other parameters.
        """
        return float(self.lat_force_interp(slip_angle))
    
    def get_longitudinal_force(self, slip_ratio: float, normal_load: Optional[float] = None,
                              tyre_pressure: Optional[float] = None,
                              temperature: Optional[float] = None) -> float:
        """
        Get longitudinal force for a given slip ratio.
        For MVP, we use simple interpolation and ignore other parameters.
        """
        return float(self.long_force_interp(slip_ratio))
    
    def get_combined_forces(self, slip_angle: float, slip_ratio: float, 
                           normal_load: Optional[float] = None) -> tuple:
        """
        Simple combined slip model that scales forces with friction circle.
        This is a very simplified approach that can be improved later.
        """
        # Pure lateral and longitudinal forces
        fy_pure = self.get_lateral_force(slip_angle)
        fx_pure = self.get_longitudinal_force(slip_ratio)
        # Simple friction circle approach
        total_force = np.sqrt(fx_pure**2 + fy_pure**2)
        # If we're below grip limit, return pure forces
        if total_force == 0:
            return (0.0, 0.0)
        # Scale forces to stay within friction circle
        max_force = max(abs(fx_pure), abs(fy_pure)) * 1.414  # Simple approximation of max force
        if total_force > max_force:
            scale = max_force / total_force
            fx = fx_pure * scale
            fy = fy_pure * scale
        else:
            fx = fx_pure
            fy = fy_pure
        return (fy, fx)


# Factory function to create appropriate tyre model based on config
def create_tyre_model(config: Dict[str, Any]) -> BaseTyreModel:
    """
    Factory function to create a tyre model based on configuration.
    
    Args:
        config: Dictionary containing configuration parameters
        
    Returns:
        An instance of a BaseTyreModel subclass
    """
    model_type = config.get('type', 'lookup')
    if model_type.lower() == 'lookup' or model_type.lower() == 'lookuptable':
        return LookupTableTyreModel.from_config(config)
    elif model_type.lower() == 'ac_lut':
        from .acLutTyre import ACLutTyreModel
        return ACLutTyreModel.from_config(config)
    # TODO: Implement PacejkaTyreModel support here
    else:
        raise ValueError(f"Unknown tyre model type: {model_type}")