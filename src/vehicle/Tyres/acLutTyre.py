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

    def __init__(self, dx_lut: pd.DataFrame, dy_lut: pd.DataFrame, base_mu: float):
        """
        Initialize AC LUT tyre model.
        Args:
            dx_lut: DataFrame with longitudinal grip multipliers vs load
            dy_lut: DataFrame with lateral grip multipliers vs load
            base_mu: Base coefficient of friction of the tire compound (default 1.5)
        """
        super().__init__()
        self.base_mu = base_mu
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
            config: Dictionary with 'file_path' (DX_LUT) and 'file_path_dy' (DY_LUT)
        Returns:
            An instance of ACLutTyreModel
        """
        filepath_dx = config.get('file_path')
        filepath_dy = config.get('file_path_dy')
        if not filepath_dx or not filepath_dy:
            raise ValueError("AC_LUT tyre model requires 'file_path' (DX_LUT) and 'file_path_dy' (DY_LUT)")
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        full_path_dx = os.path.join(project_root, filepath_dx)
        full_path_dy = os.path.join(project_root, filepath_dy)
        try:
            if not os.path.exists(full_path_dx):
                raise FileNotFoundError(f"DX_LUT file not found: {full_path_dx}")
            if not os.path.exists(full_path_dy):
                raise FileNotFoundError(f"DY_LUT file not found: {full_path_dy}")
            dx_lut = pd.read_csv(full_path_dx)
            dy_lut = pd.read_csv(full_path_dy)
            base_mu = config.get('base_mu')
            return cls(dx_lut, dy_lut, base_mu)
        except Exception as e:
            import logging
            logging.error(f"Failed to load AC LUT tyre data: {str(e)}")
            raise
            # Get optional parameters from config
            base_mu = config.get('base_mu')
            return cls(dx_lut, dy_lut, base_mu)
    
    def _get_dx_multiplier(self, normal_load: float) -> float:
        """Get normalized longitudinal grip multiplier for given load."""
        return float(self.dx_interp(normal_load))

    def _get_dy_multiplier(self, normal_load: float) -> float:
        """Get normalized lateral grip multiplier for given load."""
        return float(self.dy_interp(normal_load))

    def get_lateral_force(self, slip_angle: float, normal_load: Optional[float] = None, 
                        camber_angle: Optional[float] = None, 
                        tyre_pressure: Optional[float] = None,
                        temperature: Optional[float] = None) -> float:
        """
        Get lateral force for a given slip angle and normal load using a simplified Pacejka Magic Formula.
        """
        if normal_load is None:
            raise ValueError("normal_load must be provided. Calculate from vehicle mass and load transfer.")
        dy_mult = self._get_dy_multiplier(normal_load)
        mu_y_peak = self.base_mu * dy_mult
        fy_peak = mu_y_peak * normal_load
        b = 10.0
        c = 1.3
        d = fy_peak
        e = 0.97
        alpha_rad = np.radians(slip_angle)
        fy = d * np.sin(c * np.arctan(b * alpha_rad - e * (b * alpha_rad - np.arctan(b * alpha_rad))))
        return fy

    def get_longitudinal_force(self, slip_ratio: float, normal_load: Optional[float] = None,
                              tyre_pressure: Optional[float] = None,
                              temperature: Optional[float] = None) -> float:
        """
        Get longitudinal force for a given slip ratio and normal load using a simplified Pacejka Magic Formula.
        """
        if normal_load is None:
            raise ValueError("normal_load must be provided. Calculate from vehicle mass and load transfer.")
        dx_mult = self._get_dx_multiplier(normal_load)
        mu_x_peak = self.base_mu * dx_mult
        fx_peak = mu_x_peak * normal_load
        b = 14.5
        c = 1.5
        d = fx_peak
        e = 1.3
        kappa = slip_ratio / 100.0
        fx = d * np.sin(c * np.arctan(b * kappa - e * (b * kappa - np.arctan(b * kappa))))
        return fx

    def get_combined_forces(self, slip_angle: float, slip_ratio: float, 
                           normal_load: Optional[float] = None) -> tuple:
        """
        Get combined lateral and longitudinal forces using friction ellipse.
        """
        if normal_load is None:
            raise ValueError("normal_load must be provided. Calculate from vehicle mass and load transfer.")
        fy_pure = self.get_lateral_force(slip_angle, normal_load)
        fx_pure = self.get_longitudinal_force(slip_ratio, normal_load)
        dx_mult = self._get_dx_multiplier(normal_load)
        dy_mult = self._get_dy_multiplier(normal_load)
        fx_max = self.base_mu * dx_mult * normal_load
        fy_max = self.base_mu * dy_mult * normal_load
        if fx_max == 0 or fy_max == 0:
            return (fy_pure, fx_pure)
        ellipse_value = (fx_pure / fx_max)**2 + (fy_pure / fy_max)**2
        if ellipse_value <= 1.0:
            return (fy_pure, fx_pure)
        else:
            scale = 1.0 / np.sqrt(ellipse_value)
            fx = fx_pure * scale
            fy = fy_pure * scale
            return (fy, fx)