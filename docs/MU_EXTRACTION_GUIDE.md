# Friction Coefficient (μ) Extraction Guide for FEA Analysis

## Overview

The lap time simulator has been enhanced to extract varying μ (friction coefficient) values during acceleration, braking, and cornering. This data is essential for FEA analysis of components such as uprights, axles, and suspension components.

## What's Been Added

### 1. **Friction Coefficient Calculation**

At each point along the track, the simulator now calculates:

- **μ_Longitudinal**: Friction coefficient in the longitudinal direction (acceleration/braking)
  ```
  μ_long = |F_long_per_tyre| / F_normal_per_tyre
  ```

- **μ_Lateral**: Friction coefficient in the lateral direction (cornering)
  ```
  μ_lat = |F_lat_per_tyre| / F_normal_per_tyre
  ```

- **μ_Combined**: Total friction utilization using friction circle
  ```
  μ_combined = sqrt(μ_long² + μ_lat²)
  ```

### 2. **Force Calculations**

The simulator computes forces at each tyre contact patch:

- **Normal Load per Tyre** (currently static, can be enhanced with load transfer)
- **Longitudinal Force per Tyre** (from acceleration/braking)
- **Lateral Force per Tyre** (from cornering)

### 3. **Driving Phase Classification**

Each track point is automatically classified into:
- **Acceleration**: gLong > 0.1g, gLat < 0.3g
- **Braking**: gLong < -0.1g, gLat < 0.3g
- **Cornering**: gLat > 0.3g, |gLong| < 0.1g
- **Combined**: Significant both longitudinal and lateral acceleration
- **Cruise**: Low acceleration in both directions

## Output Files

When you run the simulator, it generates two key files in the `output/` directory:

### 1. Telemetry Data CSV (`telemetry_data_YYYYMMDD_HHMMSS.csv`)

Complete lap telemetry with columns:
- `Distance_m`: Distance along track
- `X_m`, `Y_m`, `Z_m`: Track coordinates
- `Curvature_1/m`: Track curvature
- `Speed_kph`: Vehicle speed
- `gLat_g`, `gLong_g`: Lateral and longitudinal accelerations
- `mu_Longitudinal`, `mu_Lateral`, `mu_Combined`: Friction coefficients
- `NormalLoad_per_Tyre_N`: Normal load on each tyre
- `LongForce_per_Tyre_N`, `LatForce_per_Tyre_N`: Forces on each tyre
- `DrivingPhase`: Classification of driving condition

### 2. Mu Summary Report (`mu_summary_YYYYMMDD_HHMMSS.txt`)

Statistical summary including:
- Overall maximum μ values
- Phase-specific statistics (mean, max, 95th percentile)
- **Recommended FEA Load Cases** with exact values for:
  1. Peak Braking Case
  2. Peak Acceleration Case
  3. Peak Cornering Case
  4. Peak Combined Case

## Visualization

The simulator now generates 6 plots:

1. **Speed Profile** - Original speed trace with accelerations
2. **G-G-V Diagram** - G-G diagram colored by speed
3. **Track Map (Speed)** - 2D track with speed gradient
4. **NEW: Mu Utilization Profile** - μ values along track distance
5. **NEW: Mu-Mu Diagram** - Friction ellipse visualization
6. **NEW: Track Map (Mu)** - 2D track with μ combined gradient

## Using the Data for FEA

### Step 1: Run the Simulator

```bash
python src/main.py
```

### Step 2: Review the Mu Summary

Open `output/mu_summary_YYYYMMDD_HHMMSS.txt` and identify the peak load cases.

Example output:
```
PEAK BRAKING CASE:
   Distance: 245.32 m
   gLong: -1.85 g
   mu_Long: 1.42
   Normal Load per Tyre: 750.2 N
   Long Force per Tyre: 1065.3 N
```

### Step 3: Calculate Forces for FEA

For each load case, you have:

**Total Vehicle Forces:**
```
F_long_total = gLong × mass × 9.81  (N)
F_lat_total = gLat × mass × 9.81    (N)
```

**Forces per Tyre** (from CSV):
- Read directly from `LongForce_per_Tyre_N` and `LatForce_per_Tyre_N`

**Forces at Contact Patch:**
```
F_x_tyre = LongForce_per_Tyre_N
F_y_tyre = LatForce_per_Tyre_N  
F_z_tyre = NormalLoad_per_Tyre_N
```

**Forces Through Upright/Suspension:**

The forces through the upright will depend on your suspension geometry, but the contact patch forces are:

```
F_resultant = sqrt(F_x² + F_y² + F_z²)
```

### Step 4: Apply to FEA Model

For each critical component (upright, axle, etc.):

1. **Identify the worst-case scenario** from the 4 recommended load cases
2. **Extract forces** from the summary or CSV
3. **Apply boundary conditions** in your FEA software:
   - Normal load on bearing surfaces
   - Longitudinal force through brake caliper / drivetrain
   - Lateral force through suspension arms
4. **Consider load transfer** (future enhancement) for front/rear distribution

## Advanced Usage

### Filtering Specific Conditions

Load the CSV in Python/Excel and filter by `DrivingPhase`:

```python
import pandas as df

# Load telemetry
df = pd.read_csv('output/telemetry_data_YYYYMMDD_HHMMSS.csv')

# Get all braking events
braking_data = df[df['DrivingPhase'] == 'Braking']

# Find maximum braking mu
max_brake_mu = braking_data['mu_Longitudinal'].max()
max_brake_event = braking_data[braking_data['mu_Longitudinal'] == max_brake_mu]

print(f"Max braking at distance: {max_brake_event['Distance_m'].values[0]:.2f} m")
print(f"Forces - Long: {max_brake_event['LongForce_per_Tyre_N'].values[0]:.1f} N")
```

### Analyzing Friction Circle Utilization

The `mu_Combined` value shows how close you are to the tyre's grip limit. Values approaching your `baseMu` (1.5 in the config) indicate the tyre is near its limit.

## Future Enhancements

Potential improvements:
1. **Load Transfer Modeling** - Distribute loads dynamically between front/rear and left/right
2. **Downforce Integration** - Include aerodynamic downforce in normal load calculations
3. **Per-Corner Forces** - Calculate individual corner loads instead of per-axle
4. **Safety Factors** - Automatically apply FEA safety margins
5. **Direct CAD Export** - Export force vectors in CAD-compatible formats

## Configuration

Ensure your `config.json` includes:

```json
{
  "outputDirectory": "output",
  "tyreModel": {
    "baseMu": 1.5  // Base friction coefficient of your tyres
  }
}
```

## Troubleshooting

**Q: Mu values seem too high**  
A: Check your `baseMu` value in config.json. Typical values: 1.0-1.2 (street tyres), 1.3-1.6 (racing slicks)

**Q: Forces at the tyres don't match expected values**  
A: Current version uses static weight distribution. Load transfer will be added in future updates.

**Q: Want forces at specific track locations**  
A: Use the `Distance_m` column in the CSV to find the exact point, then read all force values.

## Example FEA Workflow

1. Run simulator → Get `mu_summary_*.txt`
2. Identify "PEAK COMBINED CASE" for worst-case loading
3. Extract forces: 
   - `LongForce_per_Tyre_N` = 850 N
   - `LatForce_per_Tyre_N` = 1200 N  
   - `NormalLoad_per_Tyre_N` = 900 N
4. Apply in FEA:
   - Constrain wheel hub center
   - Apply 850N longitudinal (through brake)
   - Apply 1200N lateral (through lower arm)
   - Apply 900N normal (through upright)
5. Run analysis for stress/displacement

## Contact & Support

For questions about the μ extraction feature, refer to the simulator documentation or check the telemetry CSV for detailed point-by-point data.
