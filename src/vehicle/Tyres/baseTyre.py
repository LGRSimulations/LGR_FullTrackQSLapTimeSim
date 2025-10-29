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
    def getLateralForce(self, slipAngle: float, normalLoad: Optional[float] = None, 
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
    def getLongitudinalForce(self, slipRatio: float, normalLoad: Optional[float] = None,
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
    def getCombinedForces(self, slipAngle: float, slipRatio: float, 
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
    
    def __init__(self, tyreDataLat: pd.DataFrame, tyreDataLong: pd.DataFrame):
        """
        Initialize lookup table tyre model.
        
        Args:
            tyreDataLat: DataFrame with lateral tyre data
            tyreDataLong: DataFrame with longitudinal tyre data
        """
        super().__init__()
        self.tyreDataLat = tyreDataLat
        self.tyreDataLong = tyreDataLong

        # Create interpolation functions for lateral forces
        self.latForceInterp = interp1d(
            tyreDataLat['Slip Angle [deg]'],
            tyreDataLat['Lateral Force [N]'],
            kind='linear',
            fill_value='extrapolate'
        )
        
        # Create interpolation functions for longitudinal forces
        self.longForceInterp = interp1d(
            tyreDataLong['Slip Ratio [%]'],
            tyreDataLong['Longitudinal Force [N]'],
            kind='linear',
            fill_value='extrapolate'
        )
    
    @classmethod
    def from_config(cls, config: Dict[str, Any]):
        """
        Create a lookup table tyre model from configuration.
        
        Args:
            config: Dictionary containing configuration parameters with filepath to tyre data
            
        Returns:
            An instance of LookupTableTyreModel
        """
        filepath = config.get('filepath')
        if not filepath:
            raise ValueError("Tyre model config must specify a filepath")
        
        # Get absolute path relative to project root
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        full_path = os.path.join(project_root, filepath)
        
        # Read and process the tyre data file
        # This assumes a specific CSV format - adapt as needed for your data format
        try:
            # Check if file exists
            if not os.path.exists(full_path):
                raise FileNotFoundError(f"Tyre data file not found: {full_path}")
                
            # Read the tyre data
            if filepath.endswith('.csv'):
                # For simple CSV format
                df = pd.read_csv(full_path)
                
                # Extract lateral and longitudinal data
                # This is a simplified example - adjust based on your actual CSV structure
                if 'dataType' in df.columns:
                    # If the CSV has a column identifying data type
                    tyreDataLat = df[df['dataType'] == 'lateral'].reset_index(drop=True)
                    tyreDataLong = df[df['dataType'] == 'longitudinal'].reset_index(drop=True)
                else:
                    # If separate columns for lateral and longitudinal data
                    # Create lateral dataframe
                    tyreDataLat = pd.DataFrame({
                        'Slip Angle [deg]': df['Slip Angle [deg]'] if 'Slip Angle [deg]' in df.columns else df['SlipAngle'],
                        'Lateral Force [N]': df['Lateral Force [N]'] if 'Lateral Force [N]' in df.columns else df['LatForce'],
                        'Normal Load [N]': df['Normal Load [N]'] if 'Normal Load [N]' in df.columns else [3000] * len(df),
                        'Camber Angle [deg]': df['Camber Angle [deg]'] if 'Camber Angle [deg]' in df.columns else [0] * len(df),
                        'Tyre Pressure [kPa]': df['Tyre Pressure [kPa]'] if 'Tyre Pressure [kPa]' in df.columns else [200] * len(df),
                        'Temperature [C]': df['Temperature [C]'] if 'Temperature [C]' in df.columns else [25] * len(df)
                    })
                    
                    # Create longitudinal dataframe
                    tyreDataLong = pd.DataFrame({
                        'Slip Ratio [%]': df['Slip Ratio [%]'] if 'Slip Ratio [%]' in df.columns else df['SlipRatio'],
                        'Longitudinal Force [N]': df['Longitudinal Force [N]'] if 'Longitudinal Force [N]' in df.columns else df['LongForce'],
                        'Normal Load [N]': df['Normal Load [N]'] if 'Normal Load [N]' in df.columns else [3000] * len(df),
                        'Tyre Pressure [kPa]': df['Tyre Pressure [kPa]'] if 'Tyre Pressure [kPa]' in df.columns else [200] * len(df),
                        'Temperature [C]': df['Temperature [C]'] if 'Temperature [C]' in df.columns else [25] * len(df)
                    })
            elif filepath.endswith('.json'):
                # For JSON format
                with open(full_path, 'r') as f:
                    tyre_data = json.load(f)
                
                # Extract data based on JSON structure
                lateral_data = tyre_data.get('lateral', {})
                longitudinal_data = tyre_data.get('longitudinal', {})
                
                # Create dataframes
                tyreDataLat = pd.DataFrame({
                    'Slip Angle [deg]': lateral_data.get('slip_angles', []),
                    'Lateral Force [N]': lateral_data.get('forces', []),
                    'Normal Load [N]': lateral_data.get('normal_loads', [3000] * len(lateral_data.get('slip_angles', []))),
                    'Camber Angle [deg]': lateral_data.get('camber_angles', [0] * len(lateral_data.get('slip_angles', []))),
                    'Tyre Pressure [kPa]': lateral_data.get('pressures', [200] * len(lateral_data.get('slip_angles', []))),
                    'Temperature [C]': lateral_data.get('temperatures', [25] * len(lateral_data.get('slip_angles', [])))
                })
                
                tyreDataLong = pd.DataFrame({
                    'Slip Ratio [%]': longitudinal_data.get('slip_ratios', []),
                    'Longitudinal Force [N]': longitudinal_data.get('forces', []),
                    'Normal Load [N]': longitudinal_data.get('normal_loads', [3000] * len(longitudinal_data.get('slip_ratios', []))),
                    'Tyre Pressure [kPa]': longitudinal_data.get('pressures', [200] * len(longitudinal_data.get('slip_ratios', []))),
                    'Temperature [C]': longitudinal_data.get('temperatures', [25] * len(longitudinal_data.get('slip_ratios', [])))
                })
            else:
                raise ValueError(f"Unsupported file format: {filepath}")
                
            # Return the model
            return cls(tyreDataLat, tyreDataLong)
            
        except Exception as e:
            import logging
            logging.error(f"Failed to load tyre data from {filepath}: {str(e)}")
            raise
    
    def getLateralForce(self, slipAngle: float, normalLoad: Optional[float] = None, 
                       camberAngle: Optional[float] = None, 
                       tyrePressure: Optional[float] = None,
                       temperature: Optional[float] = None) -> float:
        """
        Get lateral force for a given slip angle.
        For MVP, we use simple interpolation and ignore other parameters.
        """
        return float(self.latForceInterp(slipAngle))
    
    def getLongitudinalForce(self, slipRatio: float, normalLoad: Optional[float] = None,
                            tyrePressure: Optional[float] = None,
                            temperature: Optional[float] = None) -> float:
        """
        Get longitudinal force for a given slip ratio.
        For MVP, we use simple interpolation and ignore other parameters.
        """
        return float(self.longForceInterp(slipRatio))
    
    def getCombinedForces(self, slipAngle: float, slipRatio: float, 
                         normalLoad: Optional[float] = None) -> tuple:
        """
        Simple combined slip model that scales forces with friction circle.
        This is a very simplified approach that can be improved later.
        """
        # Pure lateral and longitudinal forces
        Fy_pure = self.getLateralForce(slipAngle)
        Fx_pure = self.getLongitudinalForce(slipRatio)
        
        # Simple friction circle approach
        total_force = np.sqrt(Fx_pure**2 + Fy_pure**2)
        
        # If we're below grip limit, return pure forces
        if total_force == 0:
            return (0.0, 0.0)
            
        # Scale forces to stay within friction circle
        max_force = max(abs(Fx_pure), abs(Fy_pure)) * 1.414  # Simple approximation of max force
        if total_force > max_force:
            scale = max_force / total_force
            Fx = Fx_pure * scale
            Fy = Fy_pure * scale
        else:
            Fx = Fx_pure
            Fy = Fy_pure
            
        return (Fy, Fx)


# Factory function to create appropriate tyre model based on config
def createTyreModel(config: Dict[str, Any]) -> BaseTyreModel:
    """
    Factory function to create a tyre model based on configuration.
    
    Args:
        config: Dictionary containing configuration parameters
        
    Returns:
        An instance of a BaseTyreModel subclass
    """
    modelType = config.get('type', 'lookup')
    
    if modelType.lower() == 'lookup' or modelType.lower() == 'lookuptable':
        return LookupTableTyreModel.from_config(config)
    # TODO: Implement PacejkaTyreModel support here
    else:
        raise ValueError(f"Unknown tyre model type: {modelType}")