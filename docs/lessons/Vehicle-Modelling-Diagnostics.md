# Vehicle Modelling Diagnostics and Trust Checks

## Read this after
Read [Vehicle Modelling Capstone](Vehicle-Modelling.md) first.

## Why this page exists
This page helps you decide if a lap result is credible.
It also helps you decide what to tune next.

## Quick engineer checklist
1. Check what limited speed most often
2. Check corner solver confidence
3. Check friction budget behavior
4. Check load and clamp health flags
5. Check lap time integration channels

## Figure set for this page
All figures below come from one script run.

Script path [tools/analysis/generate_vehicle_modelling_diagnostics_figures.py](../../tools/analysis/generate_vehicle_modelling_diagnostics_figures.py)

Inputs

- Track = datasets/tracks/FSUK.txt
- Model variant = b1

Outputs

- limiter_mode_counts_from_sim.png
- solver_health_counters_from_sim.png
- longitudinal_budget_scale_from_sim.png
- g_channels_from_sim.png

## 1. What limited this lap
The solver logs limiter mode at each sampled point.
This gives a fast map of where performance is being constrained.

Code path

- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `forward_limiting_mode`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `backward_limiting_mode`

Current run example from FSUK with model variant b1

- Forward pass counts: `power_limited=160`, `corner_capped=97`, `initial=1`
- Backward pass counts: `corner_capped=140`, `brake_limited=117`, `terminal=1`

![Limiter mode counts from simulator](figures/vehicle_modelling_diagnostics/limiter_mode_counts_from_sim.png)

How to read this

- More `power_limited` means straight line acceleration is often power constrained
- More `brake_limited` means braking capability is often the bottleneck
- More `corner_capped` means local corner equilibrium is often setting the speed ceiling

## 2. Corner solver confidence
Pass 1 solves corner equilibrium at each point.
When it cannot converge, a constrained fallback speed is used.

Code path

- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `corner_solver_success`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `corner_fallback_used`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `corner_retry_count`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `corner_failure_reason`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `corner_tier_failure_reasons`

Current run example

- Corner solver success points: `254`
- Corner fallback points: `4`

![Solver health counters from simulator](figures/vehicle_modelling_diagnostics/solver_health_counters_from_sim.png)

Engineering takeaway

- Low fallback count is good
- Rising fallback count means setup or model inputs are pushing the corner solve outside stable regions

## 3. Friction budget coupling in accel and braking
The tyre model shares total force capacity between lateral and longitudinal demand.
As lateral demand rises, less longitudinal budget remains.

Code path

- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `forward_combined_budget_scale`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `backward_combined_budget_scale`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `_longitudinal_budget_scale_from_lateral_demand`

Core idea

$$
F_{x,available} = F_{x,cap} \cdot \text{budget\_scale}
$$

If `budget_scale` is near 1, longitudinal authority is mostly available.
If `budget_scale` drops, cornering demand is consuming more of the tyre force envelope.

![Longitudinal budget scale from simulator](figures/vehicle_modelling_diagnostics/longitudinal_budget_scale_from_sim.png)

## 4. Why drag helps braking in this model
In backward pass, drag is added as extra deceleration.
This is treated outside the tyre friction circle term.

Code path

- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `a_lat_clamped = min(a_lat, a_limit)`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `a_brake = sqrt(a_limit^2 - a_lat_clamped^2)`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `a_brake += f_drag / vehicle.params.mass`

Engineering takeaway

- Aero changes can move both top speed and braking-limited regions
- Do not evaluate drag as only a straight line penalty

## 5. Model health flags you should always check
The solver records physical guardrail events.
These tell you when the run is close to model limits.

Code path

- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `normal_load_non_physical_events_total`
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) `normal_load_transfer_clamped_events_total`

Current run example

- Non physical normal load events total: `0`
- Normal load transfer clamped events total: `229`

This same run is shown in the solver health counters figure above.

Engineering takeaway

- Non zero non physical load events are a red flag
- High clamp counts are a warning that load transfer assumptions are being stressed

## 6. How lap time is actually integrated
Lap time is integrated segment by segment from the final speed profile.
The same loop also computes longitudinal and lateral g channels.

Code path

- [src/simulator/simulator.py](../../src/simulator/simulator.py) `dt = ds / v_avg`
- [src/simulator/simulator.py](../../src/simulator/simulator.py) `g_long = ((v_curr^2 - v_prev^2) / (2*ds)) / 9.81`
- [src/simulator/simulator.py](../../src/simulator/simulator.py) `g_lat = v_curr^2 * curvature / 9.81`

Engineering takeaway

- Speed trace quality directly controls lap time quality
- Spikes in speed or bad spacing in track points can show up in g channels

Current run channel range

- Lap time = `65.353 s`
- Longitudinal g min max = `-3.367` to `+1.119`
- Lateral g min max = `-1.312` to `+1.312`

![g-channels from simulator](figures/vehicle_modelling_diagnostics/g_channels_from_sim.png)

## Suggested tuning workflow
1. Run a baseline lap and store diagnostics
2. Read limiter mode counts to identify dominant bottleneck
3. Check corner fallback and health flags before trusting small lap deltas
4. Make one subsystem change
5. Re-run and compare limiter mix plus lap time together

## Related lessons
- [Simulator Summary and Core Solver](simulator-summary.md)
- [Vehicle Modelling Capstone](Vehicle-Modelling.md)
- [Track Geometry and Sampling for Vehicle Dynamics](Track-Geometry-and-Sampling.md)
- [Aerodynamics Model Intro](Aero-Model.md)
- [Tyre Model Deep Dive](Tyre-Model-Deep-Dive.md)
- [Lessons Index](README.md)
