import pandas as pd
import numpy as np
import os
import json
from scipy.interpolate import interp1d
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import re


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
    Tyre model that uses lookup tables for force calculations with Pacejka magic formula.
    
    Flow:
    1. Parse CSV files via _parse_lateral_file / _parse_longitudinal_file
    2. Build peak force vs normal load interpolators
    3. For a given (slip, normal_load) input:
       - Interpolate peak force D at the given normal load
       - Compute B as a function of D
       - Apply Pacejka formula to get force
    """
    
    # Pacejka shape factors (can be tuned)
    C_LAT = 1.3      # Shape factor for lateral
    C_LONG = 1.65    # Shape factor for longitudinal
    E_LAT = 0.3     # Curvature factor for lateral
    E_LONG = 0.1    # Curvature factor for longitudinal
    
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

        # Build peak-vs-load interpolators (used by Pacejka D = peak)
        self._build_peak_interps()

    def _build_peak_interps(self):
        """Build peak force vs normal-load interpolators for lateral and longitudinal."""
        import logging
        logger = logging.getLogger(__name__)

        # Ensure numeric columns for lateral
        if 'Normal Load [N]' in self.tyre_data_lat.columns:
            self.tyre_data_lat['Normal Load [N]'] = pd.to_numeric(self.tyre_data_lat['Normal Load [N]'], errors='coerce')
        if 'Lateral Force [N]' in self.tyre_data_lat.columns:
            self.tyre_data_lat['Lateral Force [N]'] = pd.to_numeric(self.tyre_data_lat['Lateral Force [N]'], errors='coerce')

        # Lateral peaks: group by normal load, find max absolute force
        lat = self.tyre_data_lat.dropna(subset=['Normal Load [N]', 'Lateral Force [N]'])
        if not lat.empty:
            grp = lat.groupby('Normal Load [N]')['Lateral Force [N]'].agg(lambda s: float(np.nanmax(np.abs(s.values)))).reset_index()
            grp = grp.sort_values('Normal Load [N]').reset_index(drop=True)
            self._lat_loads = grp['Normal Load [N]'].to_numpy(dtype=float)
            self._lat_peaks = grp['Lateral Force [N]'].to_numpy(dtype=float)
            logger.debug("Lateral peak loads: %s", self._lat_loads)
            logger.debug("Lateral peak values: %s", self._lat_peaks)
            if len(self._lat_loads) >= 2:
                self._lat_peak_interp = interp1d(self._lat_loads, self._lat_peaks, kind='linear', fill_value='extrapolate', assume_sorted=True)
            elif len(self._lat_loads) == 1:
                const = float(self._lat_peaks[0])
                self._lat_peak_interp = lambda x, c=const: np.full_like(np.atleast_1d(x), c, dtype=float)
            else:
                self._lat_peak_interp = None
        else:
            self._lat_loads = np.array([])
            self._lat_peaks = np.array([])
            self._lat_peak_interp = None

        # Ensure numeric columns for longitudinal
        if 'Normal Load [N]' in self.tyre_data_long.columns:
            self.tyre_data_long['Normal Load [N]'] = pd.to_numeric(self.tyre_data_long['Normal Load [N]'], errors='coerce')
        if 'Longitudinal Force [N]' in self.tyre_data_long.columns:
            self.tyre_data_long['Longitudinal Force [N]'] = pd.to_numeric(self.tyre_data_long['Longitudinal Force [N]'], errors='coerce')

        # Longitudinal peaks: group by normal load, find max absolute force
        lon = self.tyre_data_long.dropna(subset=['Normal Load [N]', 'Longitudinal Force [N]'])
        if not lon.empty:
            grp = lon.groupby('Normal Load [N]')['Longitudinal Force [N]'].agg(lambda s: float(np.nanmax(np.abs(s.values)))).reset_index()
            grp = grp.sort_values('Normal Load [N]').reset_index(drop=True)
            self._long_loads = grp['Normal Load [N]'].to_numpy(dtype=float)
            self._long_peaks = grp['Longitudinal Force [N]'].to_numpy(dtype=float)
            logger.debug("Longitudinal peak loads: %s", self._long_loads)
            logger.debug("Longitudinal peak values: %s", self._long_peaks)
            if len(self._long_loads) >= 2:
                self._long_peak_interp = interp1d(self._long_loads, self._long_peaks, kind='linear', fill_value='extrapolate', assume_sorted=True)
            elif len(self._long_loads) == 1:
                const = float(self._long_peaks[0])
                self._long_peak_interp = lambda x, c=const: np.full_like(np.atleast_1d(x), c, dtype=float)
            else:
                self._long_peak_interp = None
        else:
            self._long_loads = np.array([])
            self._long_peaks = np.array([])
            self._long_peak_interp = None

    def _compute_B_lateral(self, D: float) -> float:
        """
        Compute B (stiffness factor) for lateral force as a function of peak force D.
        B controls how quickly the curve rises. Higher B = steeper initial slope.
        """
        if D <= 0:
            return 10.0  # default fallback
        # B scales inversely with D to keep consistent initial slope behavior
        # Tuned so that peak occurs around 6-8 degrees slip angle
        return 0.15 * 180.0 / np.pi  # ~8.6, gives peak around 6-7 deg

    def _compute_B_longitudinal(self, D: float) -> float:
        """
        Compute B (stiffness factor) for longitudinal force as a function of peak force D.
        """
        if D <= 0:
            return 10.0  # default fallback
        # For longitudinal, peak typically occurs around 10-15% slip ratio
        # With slip ratio in [0,1] (kappa), B ~ 10-15 gives peak around 0.1-0.15
        return 12.0

    def _pacejka(self, slip: float, D: float, C: float, B: float, E: float) -> float:
        """
        Pacejka Magic Formula: F = D * sin(C * arctan(B*slip - E*(B*slip - arctan(B*slip))))
        
        Args:
            slip: Slip value (radians for lateral, dimensionless ratio for longitudinal)
            D: Peak force
            C: Shape factor
            B: Stiffness factor
            E: Curvature factor
        
        Returns:
            Force in N
        """
        Bx = B * slip
        return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))

    def get_lateral_force(self, slip_angle: float, normal_load: Optional[float] = None, 
                        camber_angle: Optional[float] = None, 
                        tyre_pressure: Optional[float] = None,
                        temperature: Optional[float] = None) -> float:
        """
        Get lateral force for given slip angle and normal load.
        
        Args:
            slip_angle: Slip angle in degrees
            normal_load: Normal load in N (required for proper force calculation)
        
        Returns:
            Lateral force in N
        """
        if normal_load is None or self._lat_peak_interp is None:
            raise ValueError("Normal load is required and lateral peak interpolator must be available")
        
        # Get peak force D by interpolating at the given normal load
        D = float(self._lat_peak_interp(normal_load))
        
        # Compute B as function of D
        B = self._compute_B_lateral(D)
        
        # Convert slip angle to radians
        alpha = np.radians(float(slip_angle))
        
        # Apply Pacejka formula
        fy = self._pacejka(alpha, D, self.C_LAT, B, self.E_LAT)
        return float(fy)

    def get_longitudinal_force(self, slip_ratio: float, normal_load: Optional[float] = None,
                              tyre_pressure: Optional[float] = None,
                              temperature: Optional[float] = None) -> float:
        """
        Get longitudinal force for given slip ratio and normal load.
        
        Args:
            slip_ratio: Slip ratio in percent (0-100 range expected from data)
            normal_load: Normal load in N (required for proper force calculation)
        
        Returns:
            Longitudinal force in N
        """
        if normal_load is None or self._long_peak_interp is None:
            raise ValueError("Normal load is required and longitudinal peak interpolator must be available")
        
        # Get peak force D by interpolating at the given normal load
        D = float(self._long_peak_interp(normal_load))
        
        # Compute B as function of D
        B = self._compute_B_longitudinal(D)
        
        # Convert slip ratio from percent to dimensionless (kappa)
        kappa = float(slip_ratio) / 100.0
        
        # Apply Pacejka formula
        fx = self._pacejka(kappa, D, self.C_LONG, B, self.E_LONG)
        return float(fx)
    
    @classmethod
    def from_config(cls, config: Dict[str, Any]):
        """
        Create a lookup table tyre model from configuration.
        
        Args:
            config: Dictionary containing configuration parameters with file_path to tyre data
            
        Returns:
            An instance of LookupTableTyreModel
        """
        # Prefer separate lateral/longitudinal file paths if provided
        file_path_longit = config.get('file_path_longit')
        file_path_lateral = config.get('file_path_lateral')

        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

        try:
            # If both separate paths are provided, parse them individually
            if file_path_longit or file_path_lateral:
                tyre_data_lat = pd.DataFrame(columns=['Slip Angle [deg]', 'Lateral Force [N]', 'Normal Load [N]', 'Camber Angle [deg]', 'Tyre Pressure [kPa]', 'Temperature [C]'])
                tyre_data_long = pd.DataFrame(columns=['Slip Ratio [%]', 'Longitudinal Force [N]', 'Normal Load [N]', 'Tyre Pressure [kPa]', 'Temperature [C]'])

                if file_path_lateral:
                    full_lat = os.path.join(project_root, file_path_lateral)
                    if not os.path.exists(full_lat):
                        raise FileNotFoundError(f"Lateral tyre data file not found: {full_lat}")
                    tyre_data_lat = cls._parse_lateral_file(full_lat)

                if file_path_longit:
                    full_long = os.path.join(project_root, file_path_longit)
                    if not os.path.exists(full_long):
                        raise FileNotFoundError(f"Longitudinal tyre data file not found: {full_long}")
                    tyre_data_long = cls._parse_longitudinal_file(full_long)

                return cls(tyre_data_lat, tyre_data_long)

        except Exception as e:
            import logging
            logging.error(f"Failed to load tyre data from separate paths: {str(e)}")
            raise

        # Legacy single file path handling (for backward compatibility)
        file_path = config.get('file_path')
        if not file_path:
            raise ValueError("Tyre model config must specify file_path or file_path_longit/file_path_lateral")

        # Get absolute path relative to project root
        full_path = os.path.join(project_root, file_path)

        # Read and process the tyre data file
        # This assumes a specific CSV format - adapt as needed for your data format
        try:
            # Check if file exists
            if not os.path.exists(full_path):
                raise FileNotFoundError(f"Tyre data file not found: {full_path}")

            if file_path.endswith('.csv'):
                # Try to parse as longitudinal grouped CSV first
                tyre_data_long = cls._parse_longitudinal_file(full_path)
                # create empty lateral frame (caller may load lateral separately)
                tyre_data_lat = pd.DataFrame(columns=['Slip Angle [deg]', 'Lateral Force [N]', 'Normal Load [N]', 'Camber Angle [deg]', 'Tyre Pressure [kPa]', 'Temperature [C]'])
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
    
    def get_combined_forces(self, slip_angle: float, slip_ratio: float, 
                           normal_load: Optional[float] = None) -> tuple:
        """
        Get combined lateral and longitudinal forces using friction circle scaling.
        
        Args:
            slip_angle: Slip angle in degrees
            slip_ratio: Slip ratio in percent
            normal_load: Normal load in N (required)
        
        Returns:
            Tuple of (lateral_force, longitudinal_force) in N
        """
        if normal_load is None:
            raise ValueError("Normal load is required for combined forces calculation")
        
        # Get pure forces
        fy_pure = self.get_lateral_force(slip_angle, normal_load)
        fx_pure = self.get_longitudinal_force(slip_ratio, normal_load)
        
        # Get peak forces for friction circle limit
        D_lat = float(self._lat_peak_interp(normal_load)) if self._lat_peak_interp else abs(fy_pure)
        D_long = float(self._long_peak_interp(normal_load)) if self._long_peak_interp else abs(fx_pure)
        
        # Combined force magnitude
        total_force = np.sqrt(fx_pure**2 + fy_pure**2)
        
        if total_force == 0:
            return (0.0, 0.0)
        
        # Friction ellipse limit (approximate)
        max_combined = np.sqrt(D_lat**2 + D_long**2) * 0.9  # 90% of theoretical max
        
        if total_force > max_combined:
            scale = max_combined / total_force
            fx = fx_pure * scale
            fy = fy_pure * scale
        else:
            fx = fx_pure
            fy = fy_pure
        
        return (float(fy), float(fx))


    @staticmethod
    def _extract_number_from_label(s: str) -> Optional[float]:
        if s is None:
            return None
        s = str(s)
        m = re.search(r'([0-9]+(?:\.[0-9]+)?)', s)
        if not m:
            return None
        try:
            return float(m.group(1))
        except Exception:
            return None

    @staticmethod
    def _parse_longitudinal_file(full_path: str) -> pd.DataFrame:
        """
        Return DataFrame with columns:
         'Slip Ratio [%]', 'Longitudinal Force [N]', 'Normal Load [N]', 'Tyre Pressure [kPa]', 'Temperature [C]'
        Accepts already-parsed CSV (preferred) or grouped SR,Fx,,SR,Fx,,... raw format.
        """
        # try normal parsed CSV first
        try:
            df = pd.read_csv(full_path)
            if {'Slip Ratio [%]', 'Longitudinal Force [N]'} <= set(df.columns):
                df = df.copy()
                df['Slip Ratio [%]'] = pd.to_numeric(df['Slip Ratio [%]'], errors='coerce')
                df['Longitudinal Force [N]'] = pd.to_numeric(df['Longitudinal Force [N]'], errors='coerce')
                if 'Normal Load [N]' not in df.columns:
                    df['Normal Load [N]'] = np.nan
                if 'Tyre Pressure [kPa]' not in df.columns:
                    df['Tyre Pressure [kPa]'] = 200
                if 'Temperature [C]' not in df.columns:
                    df['Temperature [C]'] = 25
                return df[['Slip Ratio [%]', 'Longitudinal Force [N]', 'Normal Load [N]', 'Tyre Pressure [kPa]', 'Temperature [C]']]
        except Exception:
            pass

        # fallback: grouped raw CSV
        raw = pd.read_csv(full_path, header=None, dtype=str)
        if raw.shape[0] < 3:
            # empty/invalid format -> return empty frame with correct columns
            return pd.DataFrame(columns=['Slip Ratio [%]', 'Longitudinal Force [N]', 'Normal Load [N]', 'Tyre Pressure [kPa]', 'Temperature [C]'])
        load_row = raw.iloc[0].fillna('').astype(str).values
        header_row = raw.iloc[1].fillna('').astype(str).values
        data = raw.iloc[2:].reset_index(drop=True).replace('', np.nan)

        groups = []
        ncols = header_row.shape[0]
        col = 0
        while col < ncols - 1:
            h = header_row[col].strip().upper()
            h_next = header_row[col+1].strip().upper() if col+1 < ncols else ''
            if ('SR' in h or 'SLIP' in h) and ('FX' in h_next or 'F' in h_next):
                # extract normal load from same column index in load_row if present
                nl = LookupTableTyreModel._extract_number_from_label(load_row[col])
                slip_series = pd.to_numeric(data.iloc[:, col], errors='coerce')
                force_series = pd.to_numeric(data.iloc[:, col+1], errors='coerce')
                dfg = pd.DataFrame({
                    'Slip Ratio [%]': slip_series,
                    'Longitudinal Force [N]': force_series
                }).dropna(how='any')
                if dfg.empty:
                    col += 3
                    continue
                n = nl if nl is not None else np.nan
                dfg['Normal Load [N]'] = float(n) if not np.isnan(n) else np.nan
                dfg['Tyre Pressure [kPa]'] = 200
                dfg['Temperature [C]'] = 25
                groups.append(dfg[['Slip Ratio [%]', 'Longitudinal Force [N]', 'Normal Load [N]', 'Tyre Pressure [kPa]', 'Temperature [C]']])
                col += 3
                continue
            col += 1

        if groups:
            return pd.concat(groups, ignore_index=True)
        return pd.DataFrame(columns=['Slip Ratio [%]', 'Longitudinal Force [N]', 'Normal Load [N]', 'Tyre Pressure [kPa]', 'Temperature [C]'])

    @staticmethod
    def _parse_lateral_file(full_path: str) -> pd.DataFrame:
        """
        Return DataFrame with columns:
         'Slip Angle [deg]', 'Lateral Force [N]', 'Normal Load [N]', 'Camber Angle [deg]', 'Tyre Pressure [kPa]', 'Temperature [C]'
        Accepts already-parsed CSV (preferred) or grouped SA,Fy,,SA,Fy,,... raw format.
        """
        # try normal parsed CSV first
        try:
            df = pd.read_csv(full_path)
            if {'Slip Angle [deg]', 'Lateral Force [N]'} <= set(df.columns):
                df = df.copy()
                df['Slip Angle [deg]'] = pd.to_numeric(df['Slip Angle [deg]'], errors='coerce')
                df['Lateral Force [N]'] = pd.to_numeric(df['Lateral Force [N]'], errors='coerce')
                if 'Normal Load [N]' not in df.columns:
                    df['Normal Load [N]'] = np.nan
                if 'Camber Angle [deg]' not in df.columns:
                    df['Camber Angle [deg]'] = 0.0
                if 'Tyre Pressure [kPa]' not in df.columns:
                    df['Tyre Pressure [kPa]'] = 200
                if 'Temperature [C]' not in df.columns:
                    df['Temperature [C]'] = 25
                return df[['Slip Angle [deg]', 'Lateral Force [N]', 'Normal Load [N]', 'Camber Angle [deg]', 'Tyre Pressure [kPa]', 'Temperature [C]']]
        except Exception:
            pass

        # fallback: grouped raw CSV
        raw = pd.read_csv(full_path, header=None, dtype=str)
        if raw.shape[0] < 3:
            return pd.DataFrame(columns=['Slip Angle [deg]', 'Lateral Force [N]', 'Normal Load [N]', 'Camber Angle [deg]', 'Tyre Pressure [kPa]', 'Temperature [C]'])
        load_row = raw.iloc[0].fillna('').astype(str).values
        header_row = raw.iloc[1].fillna('').astype(str).values
        data = raw.iloc[2:].reset_index(drop=True).replace('', np.nan)

        groups = []
        ncols = header_row.shape[0]
        col = 0
        while col < ncols - 1:
            h = header_row[col].strip().upper()
            h_next = header_row[col+1].strip().upper() if col+1 < ncols else ''
            if ('SA' in h or 'SLIP' in h or 'S A' in h) and ('FY' in h_next or 'F' in h_next):
                nl = LookupTableTyreModel._extract_number_from_label(load_row[col])
                slip_series = pd.to_numeric(data.iloc[:, col], errors='coerce')
                force_series = pd.to_numeric(data.iloc[:, col+1], errors='coerce')
                dfg = pd.DataFrame({
                    'Slip Angle [deg]': slip_series,
                    'Lateral Force [N]': force_series
                }).dropna(how='any')
                if dfg.empty:
                    col += 3
                    continue
                n = nl if nl is not None else np.nan
                dfg['Normal Load [N]'] = float(n) if not np.isnan(n) else np.nan
                dfg['Camber Angle [deg]'] = 0.0
                dfg['Tyre Pressure [kPa]'] = 200
                dfg['Temperature [C]'] = 25
                groups.append(dfg[['Slip Angle [deg]', 'Lateral Force [N]', 'Normal Load [N]', 'Camber Angle [deg]', 'Tyre Pressure [kPa]', 'Temperature [C]']])
                col += 3
                continue
            col += 1

        if groups:
            return pd.concat(groups, ignore_index=True)
        return pd.DataFrame(columns=['Slip Angle [deg]', 'Lateral Force [N]', 'Normal Load [N]', 'Camber Angle [deg]', 'Tyre Pressure [kPa]', 'Temperature [C]'])
    
def create_tyre_model(config: Dict[str, Any]) -> BaseTyreModel:
    """
    Factory function to create a tyre model based on configuration.
    
    Args:
        config: Dictionary containing configuration parameters with 'type' key
        
    Returns:
        An instance of a tyre model
    """
    tyre_model_type = config.get('type', 'lookup').lower()
    if tyre_model_type == 'lookup':
        full_lat_path = config.get('file_path_lateral').lower()
        full_long_path = config.get('file_path_longit').lower()
        tyre_data_lat = LookupTableTyreModel._parse_lateral_file(full_lat_path)
        tyre_data_long = LookupTableTyreModel._parse_longitudinal_file(full_long_path)

        tyre = LookupTableTyreModel(tyre_data_lat, tyre_data_long)

        return tyre
    else:
        raise ValueError(f"Unsupported tyre model type: {tyre_model_type}")