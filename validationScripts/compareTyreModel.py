import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

# Import ACLutTyreModel from the src directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from vehicle.Tyres.acLutTyre import ACLutTyreModel


# --- CONFIG ---
BASE_MU = 1.5
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# Data/model paths relative to this script
MODEL_DX_LUT_PATH = os.path.join(SCRIPT_DIR, '..', 'datasets', 'vehicle', 'tyre_data', 'DX_LUT.csv')
MODEL_DY_LUT_PATH = os.path.join(SCRIPT_DIR, '..', 'datasets', 'vehicle', 'tyre_data', 'DY_LUT.csv')
TEST_LATERAL_CSV = os.path.join(SCRIPT_DIR, '..', 'datasets', 'vehicle', 'tyre_data', 'Round_8_12_PSI_Lateral_Load_TyreData.csv')
TEST_LONGITUDINAL_CSV = os.path.join(SCRIPT_DIR, '..', 'datasets', 'vehicle', 'tyre_data', 'Round_6_12_PSI_Longit_Load_TyreData.csv')

# Load model LUTs
model_DX_LUT = pd.read_csv(MODEL_DX_LUT_PATH)
model_DY_LUT = pd.read_csv(MODEL_DY_LUT_PATH)

# Create tyre model
tyre = ACLutTyreModel(model_DX_LUT, model_DY_LUT, BASE_MU)


# --- Experimental (test) Lateral Data ---
# Read the CSV as raw values (no header)
lat_raw = pd.read_csv(TEST_LATERAL_CSV, header=None)
lat_load_row = lat_raw.iloc[0].values
lat_headers = lat_raw.iloc[1].values
lat_data = lat_raw.iloc[2:].reset_index(drop=True)


# Build (load, slip_col_idx, force_col_idx, is_warm) for valid columns (robust to empty columns)
lat_col_map = []
col = 0
while col < len(lat_headers):
    header = str(lat_headers[col]).strip()
    load_str = str(lat_load_row[col]).replace('N','').replace(',','').strip()
    # Stop processing if WARMUP? marker is detected
    if 'WARMUP' in load_str.upper():
        break
    # Normal slip/force pair: 'SA [deg]' with load, next col is 'Fy [N]'
    if header.startswith('SA') and load_str:
        force_col = col + 1
        if force_col < len(lat_headers) and str(lat_headers[force_col]).strip().startswith('Fy'):
            try:
                load_val = float(load_str)
                lat_col_map.append((load_val, col, force_col, False))
            except ValueError:
                pass
        col += 2
        continue
    # Warm slip/force pair: slip_col is empty, force_col is 'Fy [N]' with load
    if header.startswith('Fy') and load_str and col > 0 and not str(lat_headers[col-1]).strip():
        try:
            load_val = float(load_str)
            lat_col_map.append((load_val, col-1, col, False))
        except ValueError:
            pass
        col += 1
        continue
    col += 1

# Print all detected normal loads for verification
print("[INFO] Detected normal loads:", [f"{load}" for load, _, _, _ in lat_col_map])


# --- Experimental (test) Longitudinal Data ---
# Read the CSV as raw values (no header)
long_raw = pd.read_csv(TEST_LONGITUDINAL_CSV, header=None)
long_load_row = long_raw.iloc[0].values
long_headers = long_raw.iloc[1].values
long_data = long_raw.iloc[2:].reset_index(drop=True)

# Build (load, sr_col_idx, fx_col_idx) for valid columns (increment by 3 for empty columns)
long_col_map = []
col = 0
while col < len(long_headers) - 1:
    load_str = str(long_load_row[col]).replace('N','').replace(',','').strip()
    sr_header = str(long_headers[col]).strip()
    fx_header = str(long_headers[col+1]).strip()
    try:
        load = float(load_str)
    except ValueError:
        col += 1
        continue  # skip non-numeric loads
    if sr_header.startswith('SR') and fx_header.startswith('Fx'):
        long_col_map.append((load, col, col+1))
    col += 3

# Print all detected normal loads for verification
print("[INFO] Detected longitudinal normal loads:", [load for load, _, _ in long_col_map])

# Create a 2x2 grid of subplots: top row 3D, bottom row 2D
fig = plt.figure(figsize=(14, 12))
ax1 = fig.add_subplot(221, projection='3d')  # 3D Lateral
ax2 = fig.add_subplot(222, projection='3d')  # 3D Longitudinal
ax3 = fig.add_subplot(223)  # 2D Lateral
ax4 = fig.add_subplot(224)  # 2D Longitudinal


# --- Lateral: Plot experimental and model ---
for i, (load, slip_idx, force_idx, _) in enumerate(lat_col_map):
    slip = pd.to_numeric(lat_data.iloc[:, slip_idx], errors='coerce').values
    fy_exp = pd.to_numeric(lat_data.iloc[:, force_idx], errors='coerce').values
    mask = ~np.isnan(slip) & ~np.isnan(fy_exp)
    slip = slip[mask]
    fy_exp = fy_exp[mask]
    fy_model = np.array([tyre.get_lateral_force(sa, normalLoad=load) for sa in slip])
    label_test = f"Test {int(load)}N"
    label_model = f"Model {int(load)}N"
    ax1.plot(slip, [load]*len(slip), fy_exp, 'o', label=label_test if i==0 else "")
    ax1.plot(slip, [load]*len(slip), fy_model, '-', label=label_model if i==0 else "")
    # 2D plot
    ax3.plot(slip, fy_exp, 'o', label=label_test if i==0 else "")
    ax3.plot(slip, fy_model, '-', label=label_model if i==0 else "")

ax1.set_xlabel('Slip Angle (deg)')
ax1.set_ylabel('Normal Load (N)')
ax1.set_zlabel('Lateral Force Fy (N)')
ax1.set_title('Lateral: Exp vs Model')
ax1.legend()

ax3.set_xlabel('Slip Angle (deg)')
ax3.set_ylabel('Lateral Force Fy (N)')
ax3.set_title('Lateral: Exp vs Model (2D)')
ax3.legend()

# --- Longitudinal: Plot experimental and model ---
for i, (load, sr_idx, fx_idx) in enumerate(long_col_map):
    sr = pd.to_numeric(long_data.iloc[:, sr_idx], errors='coerce').values
    fx_exp = pd.to_numeric(long_data.iloc[:, fx_idx], errors='coerce').values
    mask = ~np.isnan(sr) & ~np.isnan(fx_exp)
    sr = sr[mask]
    fx_exp = fx_exp[mask]
    fx_model = np.array([tyre.get_longitudinal_force(s*100, normalLoad=load) for s in sr])  # s*100 as SR is fraction
    label_test = f"Test {int(load)}N"
    label_model = f"Model {int(load)}N"
    ax2.plot(sr, [load]*len(sr), fx_exp, 'o', label=label_test if i==0 else "")
    ax2.plot(sr, [load]*len(sr), fx_model, '-', label=label_model if i==0 else "")
    # 2D plot
    ax4.plot(sr, fx_exp, 'o', label=label_test if i==0 else "")
    ax4.plot(sr, fx_model, '-', label=label_model if i==0 else "")

ax2.set_xlabel('Slip Ratio')
ax2.set_ylabel('Normal Load (N)')
ax2.set_zlabel('Longitudinal Force Fx (N)')
ax2.set_title('Longitudinal: Exp vs Model')
ax2.legend()

ax4.set_xlabel('Slip Ratio')
ax4.set_ylabel('Longitudinal Force Fx (N)')
ax4.set_title('Longitudinal: Exp vs Model (2D)')
ax4.legend()

plt.tight_layout()
plt.show()
