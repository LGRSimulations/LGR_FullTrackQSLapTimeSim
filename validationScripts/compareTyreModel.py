import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

# Import Look Up table tyre model from the src directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from vehicle.Tyres.baseTyre import LookupTableTyreModel

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VEHICLE_MASS = 320  # Kg, including driver
REPRESENTATIVE_NORMAL_LOAD = VEHICLE_MASS * 9.81 / 4  # N, per wheel

TEST_LATERAL_CSV = os.path.join(SCRIPT_DIR, '..', 'datasets', 'vehicle', 'tyre_data', 'Round_8_12_PSI_Lateral_Load_TyreData.csv')
TEST_LONGITUDINAL_CSV = os.path.join(SCRIPT_DIR, '..', 'datasets', 'vehicle', 'tyre_data', 'Round_6_12_PSI_Longit_Load_TyreData.csv')

# Parse tyre CSVs with the LookupTableTyreModel helpers and instantiate model
full_lat_path = os.path.abspath(TEST_LATERAL_CSV)
full_long_path = os.path.abspath(TEST_LONGITUDINAL_CSV)

tyre_data_lat = LookupTableTyreModel._parse_lateral_file(full_lat_path)
tyre_data_long = LookupTableTyreModel._parse_longitudinal_file(full_long_path)

tyre = LookupTableTyreModel(tyre_data_lat, tyre_data_long)

# --- Test Lateral Data ---
test_lat_raw = pd.read_csv(TEST_LATERAL_CSV, header=None)
test_lat_load_row = test_lat_raw.iloc[0].values
test_lat_headers = test_lat_raw.iloc[1].values
test_lat_data = test_lat_raw.iloc[2:].reset_index(drop=True)

test_lateral_col_map = []
col_idx = 0
while col_idx < len(test_lat_headers):
    header = str(test_lat_headers[col_idx]).strip()
    normal_load_str = str(test_lat_load_row[col_idx]).replace('N','').replace(',','').strip()
    if 'WARMUP' in normal_load_str.upper():
        break
    if header.startswith('SA') and normal_load_str:
        lateral_force_col_idx = col_idx + 1
        if lateral_force_col_idx < len(test_lat_headers) and str(test_lat_headers[lateral_force_col_idx]).strip().startswith('Fy'):
            try:
                normal_load_val = float(normal_load_str)
                test_lateral_col_map.append((normal_load_val, col_idx, lateral_force_col_idx, False))
            except ValueError:
                pass
        col_idx += 2
        continue
    if header.startswith('Fy') and normal_load_str and col_idx > 0 and not str(test_lat_headers[col_idx-1]).strip():
        try:
            normal_load_val = float(normal_load_str)
            test_lateral_col_map.append((normal_load_val, col_idx-1, col_idx, False))
        except ValueError:
            pass
        col_idx += 1
        continue
    col_idx += 1
print("[INFO] Detected lateral normal loads:", [f"{normal_load}" for normal_load, _, _, _ in test_lateral_col_map])

# --- Test Longitudinal Data ---
test_long_raw = pd.read_csv(TEST_LONGITUDINAL_CSV, header=None)
test_long_load_row = test_long_raw.iloc[0].values
test_long_headers = test_long_raw.iloc[1].values
test_long_data = test_long_raw.iloc[2:].reset_index(drop=True)

test_longitudinal_col_map = []
col_idx = 0
while col_idx < len(test_long_headers) - 1:
    normal_load_str = str(test_long_load_row[col_idx]).replace('N','').replace(',','').strip()
    slip_ratio_header = str(test_long_headers[col_idx]).strip()
    longitudinal_force_header = str(test_long_headers[col_idx+1]).strip()
    try:
        normal_load_val = float(normal_load_str)
    except ValueError:
        col_idx += 1
        continue
    if slip_ratio_header.startswith('SR') and longitudinal_force_header.startswith('Fx'):
        test_longitudinal_col_map.append((normal_load_val, col_idx, col_idx+1))
    col_idx += 3
print("[INFO] Detected longitudinal normal loads:", [normal_load for normal_load, _, _ in test_longitudinal_col_map])

fig = plt.figure(figsize=(14, 12))
ax1 = fig.add_subplot(221, projection='3d')
ax2 = fig.add_subplot(222, projection='3d')
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)

# --- Lateral: Plot test and model ---
for i, (normal_load, slip_angle_col_idx, lateral_force_col_idx, _) in enumerate(test_lateral_col_map):
    slip_angle_deg = pd.to_numeric(test_lat_data.iloc[:, slip_angle_col_idx], errors='coerce').values
    lateral_force_test = pd.to_numeric(test_lat_data.iloc[:, lateral_force_col_idx], errors='coerce').values
    valid_mask = ~np.isnan(slip_angle_deg) & ~np.isnan(lateral_force_test)
    slip_angle_deg = slip_angle_deg[valid_mask]
    lateral_force_test = lateral_force_test[valid_mask]
    lateral_force_model = np.array([tyre.get_lateral_force(sa, normal_load=normal_load) for sa in slip_angle_deg])
    label_test = f"Test {int(normal_load)}N"
    label_model = f"Model {int(normal_load)}N"
    ax1.plot(slip_angle_deg, [normal_load]*len(slip_angle_deg), lateral_force_test, 'o', label=label_test if i==0 else "")
    ax1.plot(slip_angle_deg, [normal_load]*len(slip_angle_deg), lateral_force_model, '-', label=label_model if i==0 else "")
    ax3.plot(slip_angle_deg, lateral_force_test, 'o', label=label_test if i==0 else "")
    ax3.plot(slip_angle_deg, lateral_force_model, '-', label=label_model if i==0 else "")

ax1.set_xlabel('Slip Angle (deg)')
ax1.set_ylabel('Normal Load (N)')
ax1.set_zlabel('Lateral Force Fy (N)')
ax1.set_title('Lateral: Exp vs Model')
ax1.legend()

ax3.set_xlabel('Slip Angle (deg)')
ax3.set_ylabel('Lateral Force Fy (N)')
ax3.set_title('Lateral: Exp vs Model (2D)')
ax3.legend()

# --- Longitudinal: Plot test and model ---
for i, (normal_load, slip_ratio_col_idx, longitudinal_force_col_idx) in enumerate(test_longitudinal_col_map):
    slip_ratio = pd.to_numeric(test_long_data.iloc[:, slip_ratio_col_idx], errors='coerce').values
    longitudinal_force_test = pd.to_numeric(test_long_data.iloc[:, longitudinal_force_col_idx], errors='coerce').values
    valid_mask = ~np.isnan(slip_ratio) & ~np.isnan(longitudinal_force_test)
    slip_ratio = slip_ratio[valid_mask]
    longitudinal_force_test = longitudinal_force_test[valid_mask]
    longitudinal_force_model = np.array([tyre.get_longitudinal_force(sr*100, normal_load=normal_load) for sr in slip_ratio])
    label_test = f"Test {int(normal_load)}N"
    label_model = f"Model {int(normal_load)}N"
    ax2.plot(slip_ratio, [normal_load]*len(slip_ratio), longitudinal_force_test, 'o', label=label_test if i==0 else "")
    ax2.plot(slip_ratio, [normal_load]*len(slip_ratio), longitudinal_force_model, '-', label=label_model if i==0 else "")
    ax4.plot(slip_ratio, longitudinal_force_test, 'o', label=label_test if i==0 else "")
    ax4.plot(slip_ratio, longitudinal_force_model, '-', label=label_model if i==0 else "")

ax2.set_xlabel('Slip Ratio')
ax2.set_ylabel('Normal Load (N)')
ax2.set_zlabel('Longitudinal Force Fx (N)')
ax2.set_title('Longitudinal: Exp vs Model')
ax2.legend()

ax4.set_xlabel('Slip Ratio')
ax4.set_ylabel('Longitudinal Force Fx (N)')
ax4.set_title('Longitudinal: Exp vs Model (2D)')
ax4.legend()

print("\n[INFO] Zero-Zero Test Results:")
vehicle_normal_load = REPRESENTATIVE_NORMAL_LOAD
zero_tests = [
    ("Lateral force at 0 slip angle, 0 normal load", tyre.get_lateral_force(0, normal_load=0)),
    ("Lateral force at 0 slip angle, 200N normal load", tyre.get_lateral_force(0, normal_load=200)),
    (f"Lateral force at 0 slip angle, vehicle normal load ({vehicle_normal_load:.1f}N)", tyre.get_lateral_force(0, normal_load=vehicle_normal_load)),
    ("Longitudinal force at 0 slip ratio, 0 normal load", tyre.get_longitudinal_force(0, normal_load=0)),
    ("Longitudinal force at 0 slip ratio, 200N normal load", tyre.get_longitudinal_force(0, normal_load=200)),
    (f"Longitudinal force at 0 slip ratio, vehicle normal load ({vehicle_normal_load:.1f}N)", tyre.get_longitudinal_force(0, normal_load=vehicle_normal_load)),
    ("Lateral force at 10 deg slip angle, 0 normal load", tyre.get_lateral_force(10, normal_load=0)),
    ("Longitudinal force at 10% slip ratio, 0 normal load", tyre.get_longitudinal_force(10, normal_load=0)),
]
for desc, val in zero_tests:
    print(f"  {desc}: {val:.4f}")


plt.show()
