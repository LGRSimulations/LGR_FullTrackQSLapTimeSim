import pandas as pd
import numpy as np
import os
from scipy.interpolate import interp1d
from typing import Dict, Any, Optional
from .baseTyre import BaseTyreModel


class ACLutTyreModel(BaseTyreModel):
    """
    Assetto Corsa style lookup table tyre model.
    Uses normalized grip multipliers (DX and DY) based on normal load.
    """
    
    def __init__(self, dx_lut: pd.DataFrame, dy_lut: pd.DataFrame, 
                 baseMu: float):
        """
        Initialize AC LUT tyre model.
        
        Args:
            dx_lut: DataFrame with longitudinal grip multipliers vs load
            dy_lut: DataFrame with lateral grip multipliers vs load
            baseMu: Base coefficient of friction of the tire compound (default 1.5)
        """
        super().__init__()
        
        self.baseMu = baseMu
        
        # Create interpolation functions for normalized grip multipliers
        self.dx_interp = interp1d(
            dx_lut.iloc[:, 0],  # First column: Nominal tyre load
            dx_lut.iloc[:, 1],  # Second column: Normalized longitudinal grip
            kind='linear',
            fill_value='extrapolate'
        )
        
        self.dy_interp = interp1d(
            dy_lut.iloc[:, 0],  # First column: Nominal tyre load
            dy_lut.iloc[:, 1],  # Second column: Normalized lateral grip
            kind='linear',
            fill_value='extrapolate'
        )
    
    @classmethod
    def from_config(cls, config: Dict[str, Any]):
        """
        Create an AC LUT tyre model from configuration.
        
        Args:
            config: Dictionary with 'filepath' (DX_LUT) and 'filepath2' (DY_LUT)
            
        Returns:
            An instance of ACLutTyreModel
        """
        filepath_dx = config.get('filepath')
        filepath_dy = config.get('filepath2')
        
        if not filepath_dx or not filepath_dy:
            raise ValueError("AC_LUT tyre model requires 'filepath' (DX_LUT) and 'filepath2' (DY_LUT)")
        
        # Get absolute paths relative to project root
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        full_path_dx = os.path.join(project_root, filepath_dx)
        full_path_dy = os.path.join(project_root, filepath_dy)
        
        try:
            # Check if files exist
            if not os.path.exists(full_path_dx):
                raise FileNotFoundError(f"DX_LUT file not found: {full_path_dx}")
            if not os.path.exists(full_path_dy):
                raise FileNotFoundError(f"DY_LUT file not found: {full_path_dy}")
            
            # Read the lookup tables
            dx_lut = pd.read_csv(full_path_dx)
            dy_lut = pd.read_csv(full_path_dy)
            
            # Get optional parameters from config
            baseMu = config.get('baseMu')

            return cls(dx_lut, dy_lut, baseMu)
            
        except Exception as e:
            import logging
            logging.error(f"Failed to load AC LUT tyre data: {str(e)}")
            raise
    
    def _get_dx_multiplier(self, normalLoad: float) -> float:
        """Get normalized longitudinal grip multiplier for given load."""
        return float(self.dx_interp(normalLoad))
    
    def _get_dy_multiplier(self, normalLoad: float) -> float:
        """Get normalized lateral grip multiplier for given load."""
        return float(self.dy_interp(normalLoad))
    
    def getLateralForce(self, slipAngle: float, normalLoad: Optional[float] = None, 
                    camberAngle: Optional[float] = None, 
                    tyrePressure: Optional[float] = None,
                    temperature: Optional[float] = None) -> float:
        """
        Get lateral force for a given slip angle and normal load using a simplified Pacejka Magic Formula.
        """
        if normalLoad is None:
            raise ValueError("normalLoad must be provided. Calculate from vehicle mass and load transfer.")

        # Get normalized lateral grip multiplier
        dy_mult = self._get_dy_multiplier(normalLoad)
        mu_y_peak = self.baseMu * dy_mult
        Fy_peak = mu_y_peak * normalLoad

        # --- Pacejka parameters ---
        B = 10.0        # stiffness factor (fit to match cornering stiffness)
        C = 1.3         # shape factor (controls "roundness" of the curve)
        D = Fy_peak     # peak value (from your model)
        E = 0.97        # curvature factor (controls drop-off after peak)

        # Convert slip angle to radians for Pacejka formula
        alpha_rad = np.radians(slipAngle)

        # Simplified Pacejka Magic Formula (no horizontal/vertical shifts)
        Fy = D * np.sin(C * np.arctan(B * alpha_rad - E * (B * alpha_rad - np.arctan(B * alpha_rad))))

        return Fy
    
    def getLongitudinalForce(self, slipRatio: float, normalLoad: Optional[float] = None,
                            tyrePressure: Optional[float] = None,
                            temperature: Optional[float] = None) -> float:
        """
        Get longitudinal force for a given slip ratio and normal load using a simplified Pacejka Magic Formula.
        
        Args:
            slipRatio: Slip ratio in percent (e.g., 10 for 10%)
            normalLoad: Normal load in N (REQUIRED - must be provided from vehicle dynamics)
            tyrePressure: Tyre pressure in kPa (ignored for now)
            temperature: Tyre temperature in °C (ignored for now)
            
        Returns:
            Longitudinal force in N
        """
        if normalLoad is None:
            raise ValueError("normalLoad must be provided. Calculate from vehicle mass and load transfer.")
        
        # Get normalized longitudinal grip multiplier
        dx_mult = self._get_dx_multiplier(normalLoad)
        mu_x_peak = self.baseMu * dx_mult
        Fx_peak = mu_x_peak * normalLoad

        # --- Pacejka parameters for longitudinal ---
        # You should fit these to your data for best results!
        B = 14.5  # stiffness factor (fit to match initial slope)
        C = 1.5   # shape factor
        D = Fx_peak  # peak value
        E = 1.3  # curvature factor

        # Convert slip ratio to a fraction (data is a percent)
        kappa = slipRatio / 100.0

        # Simplified Pacejka Magic Formula (no horizontal/vertical shifts)
        Fx = D * np.sin(C * np.arctan(B * kappa - E * (B * kappa - np.arctan(B * kappa))))

        return Fx
    
    def getCombinedForces(self, slipAngle: float, slipRatio: float, 
                         normalLoad: Optional[float] = None) -> tuple:
        """
        Get combined lateral and longitudinal forces using friction ellipse.
        
        Args:
            slipAngle: Slip angle in degrees
            slipRatio: Slip ratio in percent
            normalLoad: Normal load in N (REQUIRED - must be provided from vehicle dynamics)
            
        Returns:
            Tuple of (lateral_force, longitudinal_force) in N
        """
        if normalLoad is None:
            raise ValueError("normalLoad must be provided. Calculate from vehicle mass and load transfer.")
        
        # Get pure forces
        Fy_pure = self.getLateralForce(slipAngle, normalLoad)
        Fx_pure = self.getLongitudinalForce(slipRatio, normalLoad)
        
        # Get peak forces for this load
        dx_mult = self._get_dx_multiplier(normalLoad)
        dy_mult = self._get_dy_multiplier(normalLoad)
        
        Fx_max = self.baseMu * dx_mult * normalLoad
        Fy_max = self.baseMu * dy_mult * normalLoad
        
        # Friction ellipse approach
        # (Fx/Fx_max)^2 + (Fy/Fy_max)^2 <= 1
        
        if Fx_max == 0 or Fy_max == 0:
            return (Fy_pure, Fx_pure)
        
        # Check if we're within the ellipse
        ellipse_value = (Fx_pure / Fx_max)**2 + (Fy_pure / Fy_max)**2
        
        if ellipse_value <= 1.0:
            # Within grip limit
            return (Fy_pure, Fx_pure)
        else:
            # Scale back to ellipse boundary
            scale = 1.0 / np.sqrt(ellipse_value)
            Fx = Fx_pure * scale
            Fy = Fy_pure * scale
            return (Fy, Fx)