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

        # keep simple 1D interpolants for legacy usage (no normal load)
        self.lat_force_interp = interp1d(
            tyre_data_lat['Slip Angle [deg]'],
            tyre_data_lat['Lateral Force [N]'],
            kind='linear',
            fill_value='extrapolate',
            assume_sorted=False
        )
        self.long_force_interp = interp1d(
            tyre_data_long['Slip Ratio [%]'],
            tyre_data_long['Longitudinal Force [N]'],
            kind='linear',
            fill_value='extrapolate',
            assume_sorted=False
        )

        # Build peak-vs-load interpolators (used by Pacejka D = peak)
        self._build_peak_interps()

    def _build_peak_interps(self):
        """Build simple peak force vs normal-load interpolators for lat and long."""
        # Lateral peaks
        lat = self.tyre_data_lat.dropna(subset=['Normal Load [N]', 'Lateral Force [N]'])
        if not lat.empty:
            grp = lat.groupby('Normal Load [N]')['Lateral Force [N]'].apply(lambda s: float(np.nanmax(np.abs(s.values)))).reset_index()
            loads = grp['Normal Load [N]'].values.astype(float)
            peaks = grp['Lateral Force [N]'].values.astype(float)
            if len(loads) >= 2:
                self._lat_peak_interp = interp1d(loads, peaks, kind='linear', fill_value='extrapolate', assume_sorted=False)
            elif len(loads) == 1:
                const = float(peaks[0])
                self._lat_peak_interp = lambda x, c=const: np.full_like(np.atleast_1d(x), c, dtype=float)
            else:
                self._lat_peak_interp = None
        else:
            self._lat_peak_interp = None

        # Longitudinal peaks
        lon = self.tyre_data_long.dropna(subset=['Normal Load [N]', 'Longitudinal Force [N]'])
        if not lon.empty:
            grp = lon.groupby('Normal Load [N]')['Longitudinal Force [N]'].apply(lambda s: float(np.nanmax(np.abs(s.values)))).reset_index()
            loads = grp['Normal Load [N]'].values.astype(float)
            peaks = grp['Longitudinal Force [N]'].values.astype(float)
            if len(loads) >= 2:
                self._long_peak_interp = interp1d(loads, peaks, kind='linear', fill_value='extrapolate', assume_sorted=False)
            elif len(loads) == 1:
                const = float(peaks[0])
                self._long_peak_interp = lambda x, c=const: np.full_like(np.atleast_1d(x), c, dtype=float)
            else:
                self._long_peak_interp = None
        else:
            self._long_peak_interp = None

    def _estimate_B_from_small_slip(self, df: pd.DataFrame, load: float,
                                    slip_col: str, force_col: str,
                                    small_window: float = 1.0, C: float = 1.3, D: float = None,
                                    is_longitudinal: bool = False) -> float:
        """
        Estimate B from the small-slip slope for supplied load.
        - df: DataFrame with columns slip_col, force_col, 'Normal Load [N]'.
        - small_window: degrees (lat) or percent (long) window around zero used for slope fit.
        - is_longitudinal: True => slip_col is slip ratio units (same unit as stored).
        Returns a B value (fallback to default if not enough data).
        """
        if D is None or D == 0:
            return 10.0  # fallback
        # Find nearest load group
        if 'Normal Load [N]' in df.columns:
            available = df['Normal Load [N]'].dropna().unique().astype(float)
            if len(available) == 0:
                return 10.0
            nearest = float(available[np.argmin(np.abs(available - load))])
            subset = df[np.isclose(df['Normal Load [N]'].astype(float), nearest, atol=1e-6)]
        else:
            subset = df.copy()

        if subset.empty:
            return 10.0

        # select small-slip rows
        slip_vals = pd.to_numeric(subset[slip_col], errors='coerce')
        force_vals = pd.to_numeric(subset[force_col], errors='coerce')
        mask = slip_vals.abs() <= small_window
        slip_small = slip_vals[mask].dropna().values
        force_small = force_vals[mask].dropna().values
        if len(slip_small) < 2:
            return 10.0

        # Linear fit slope (force vs slip). For lateral slip_col in degrees -> convert slope to N/rad
        p = np.polyfit(slip_small.astype(float), force_small.astype(float), 1)
        slope = float(p[0])  # N per unit-of-slip_col

        if is_longitudinal:
            # slope is N per percent (if slip stored in percent) -> convert to N per unit kappa (kappa = slip/100)
            slope_per_kappa = slope / 100.0
            slope_rad = slope_per_kappa  # use as N per kappa
        else:
            # lateral: slope is N / deg -> convert to N / rad
            slope_rad = slope * (180.0 / np.pi)

        # Pacejka small-angle approx: initial_slope ≈ C * B * D  => B = slope_rad / (C * D)
        if D == 0:
            return 10.0
        B = slope_rad / (C * D)
        if not np.isfinite(B) or B <= 0:
            return 10.0
        return float(B)

    def get_lateral_force(self, slip_angle: float, normal_load: Optional[float] = None, 
                        camber_angle: Optional[float] = None, 
                        tyre_pressure: Optional[float] = None,
                        temperature: Optional[float] = None) -> float:
        """
        Get lateral force — if normal_load provided, use Pacejka with D from peak-interp.
        Otherwise use 1D lookup.
        """
        # If normal load provided and we have peak interpolation, use Pacejka
        if normal_load is not None and self._lat_peak_interp is not None:
            D = float(self._lat_peak_interp(normal_load))
            C = 1.3
            E = 0.97
            # estimate B from small-slip slope near this load (fallback to default)
            B = self._estimate_B_from_small_slip(self.tyre_data_lat, normal_load,
                                                 slip_col='Slip Angle [deg]',
                                                 force_col='Lateral Force [N]',
                                                 small_window=1.0, C=C, D=D, is_longitudinal=False)
            alpha = np.radians(float(slip_angle))
            fy = D * np.sin(C * np.arctan(B * alpha - E * (B * alpha - np.arctan(B * alpha))))
            return float(fy)
        # fallback to naive interpolation by slip angle
        return float(self.lat_force_interp(slip_angle))

    def get_longitudinal_force(self, slip_ratio: float, normal_load: Optional[float] = None,
                              tyre_pressure: Optional[float] = None,
                              temperature: Optional[float] = None) -> float:
        """
        Get longitudinal force — if normal_load provided, use Pacejka with D from peak-interp.
        Otherwise use 1D lookup.
        """
        if normal_load is not None and self._long_peak_interp is not None:
            D = float(self._long_peak_interp(normal_load))
            C = 1.5
            E = 1.3
            # estimate B from small-slip slope near this load (slip ratio units, treat as percent)
            B = self._estimate_B_from_small_slip(self.tyre_data_long, normal_load,
                                                 slip_col='Slip Ratio [%]',
                                                 force_col='Longitudinal Force [N]',
                                                 small_window=1.0, C=C, D=D, is_longitudinal=True)
            # convert slip_ratio to normalized kappa used by Pacejka model (kappa = slip_ratio/100)
            kappa = float(slip_ratio) / 100.0
            fx = D * np.sin(C * np.arctan(B * kappa - E * (B * kappa - np.arctan(B * kappa))))
            return float(fx)
        return float(self.long_force_interp(slip_ratio))
    
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
# ...existing code...