# Vehicle Modelling Capstone

## Before you start
Read [Aerodynamics Model Intro](Aero-Model.md) first.

## What you will learn
- What each pass does
- How the final speed is chosen
- Why the black line can look lower than every colored line
- Where to check the code and tests

## High-level overview of the sim
At each sampled track point, the solver asks three questions.

1. Pass 1 asks what corner equilibrium allows at this point
2. Pass 2 asks what acceleration can reach from the previous point
3. Pass 3 asks what braking must allow for the next points

The final speed takes the lowest allowed value.

$$
v_{final,i} = \min\left(v_{pass1,i},\ v_{pass2,i},\ v_{pass3,i}\right)
$$

If you only remember one rule, remember this one.
The final speed is always the most restrictive limit at each sampled point.

Implementation truth in code

- Pass 1 is applied as a corner cap inside Pass 2 and Pass 3
- The stored final profile is then computed as $\min(v_{pass2}, v_{pass3})$ point by point

Think of Pass 1 as a ceiling.
Then Pass 2 and Pass 3 push upward from each side.
The lower surface becomes the final lap profile.

## The sim visually
![Three pass speed envelope from simulator](figures/vehicle_modelling/three_pass_speed_envelope_from_sim.png)

Script path [tools/analysis/generate_vehicle_modelling_capstone_figures.py](../../tools/analysis/generate_vehicle_modelling_capstone_figures.py)

Inputs

- Track = datasets/tracks/FSUK.txt
- Model variant = b1
- Solver flow = Pass 1 corner ceiling, Pass 2 forward accel, Pass 3 backward brake

Results

- Final profile in code is the min of Pass 2 and Pass 3 after both are corner capped by Pass 1
- Worked segment index = 41 at distance 213.93 m
- Lap time from this run = 65.353 s

## Worked example
This point shows why backward braking limits matter.
It is near straight curvature but it still must satisfy upcoming braking needs.

Inputs

- Segment index = 41
- Distance = 213.93 m
- Curvature = 4.441e-09 1/m

Results

- Pass 1 speed = 200.00 m/s
- Pass 2 speed = 33.30 m/s
- Pass 3 speed = 16.56 m/s
- Final speed = 16.56 m/s
- Limiting pass = Pass 3

At this segment

$$
v_{final} = \min(200.00,\ 33.30,\ 16.56) = 16.56\ \text{m/s}
$$

Equivalent implementation form at this same segment

$$
v_{final} = \min(33.30,\ 16.56) = 16.56\ \text{m/s}
$$

That is the key behavior of the three pass method.

## Why the black line can look lower than every pass

The solver computes speed at discrete sampled points.
At each sampled point, the final speed is an exact minimum.
There is no hidden post filter in the combine step.

$$
v_{final,i} = \min(v_{pass2,i}, v_{pass3,i})
$$

Pass 1 is already applied inside Pass 2 and Pass 3 through corner caps, so the final combine takes the min of those two constrained profiles.

The visual effect appears between points when the active limiter switches.

A limiter is the rule that currently sets the lowest allowed speed.
A limiter handoff means that this rule changes from one pass to another between nearby points.

Example from this run around one segment pair

- At point i, Pass 2 = 24.78 and Pass 3 = 35.84 so final picks 24.78
- At point i+1, Pass 2 = 28.50 and Pass 3 = 22.85 so final picks 22.85

On the chart, each curve is drawn by joining sampled points with straight lines.
During a handoff, the black line joins one point limited by Pass 2 to the next point limited by Pass 3.
That joined black segment can sit below both colored segments near the middle, even though each sampled point is still an exact pointwise minimum.

So when black appears below all colored lines locally, that is a line-joining artifact from limiter handoff between adjacent sample points. It is not an extra realism filter.

This joined line is only a plotting choice.
The solver limits are enforced at the sampled points.
Lap time integration also uses these sampled node speeds.

## What happens near the start and end points

Forward and backward passes start from different ends of the point list.
Forward starts at point 0 with `speeds[0] = point_speeds[0]`.
Backward starts at the last point with `speeds[-1] = point_speeds[-1]`.

Because of this, the active limiter near the boundaries can change with track and setup.
There is no fixed rule that backward must be lower at the end.

In this current run, the last point is forward limited.
At index 257, Pass 2 is 15.81 m/s and Pass 3 is 16.20 m/s.
So the final speed follows Pass 2 at that point.

The pass loops run along the listed points only.
They do not add an extra wraparound segment from last point back to first point inside these loops.

## Deeper dive in each pass

### Pass 1 corner limit
Pass 1 solves each point for the highest feasible corner speed.
It uses lateral and yaw equilibrium with tyre limits.

Core relation

$$
a_y = v^2\kappa
$$

Code path

- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) function `optimise_speed_at_points`
- [src/simulator/util/vehicleDynamics.py](../../src/simulator/util/vehicleDynamics.py) function `find_vehicle_state_at_point`

### Pass 2 forward acceleration limit
Pass 2 walks from start to end.
It applies powertrain request, tyre limits, and drag.

Core relations

$$
F_{x,net} = \min(F_{x,power},\ F_{x,tyre\ limit}) - F_{drag}
$$

$$
v_{pass2,i} = \min\left(v_{pass1,i},\ \sqrt{v_{pass2,i-1}^2 + 2a_x\Delta s}\right)
$$

Code path

- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) function `forward_pass`

### Pass 3 backward braking limit
Pass 3 walks from end to start.
It enforces braking feasibility for upcoming segments.

Core relation

$$
v_{pass3,i} = \min\left(v_{pass1,i},\ \sqrt{v_{pass3,i+1}^2 + 2a_{brake}\Delta s}\right)
$$

Code path

- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) function `backward_pass`

### Final combine and lap time
The final speed profile takes the minimum of forward and backward constrained profiles.

Code path

- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) function `compute_speed_profile`
- [src/simulator/simulator.py](../../src/simulator/simulator.py) function `run_lap_time_simulation`

## How the code runs step by step

1. Build track and vehicle models
2. Run Pass 1 for point wise corner ceilings
3. Run Pass 2 for forward acceleration propagation
4. Run Pass 3 for backward braking propagation
5. Combine profiles and integrate lap time

## Assumptions and limits
- Quasi static segment level model
- Steady state corner solve at each point
- If corner equilibrium solve fails at a point, a constrained fallback speed is used and logged
- No full suspension transient states in this solver
- No full tyre thermal transient states in this solver

## How to verify this yourself
- [tests/test_solver_contracts.py](../../tests/test_solver_contracts.py)
- [tests/test_tyre_force_contracts.py](../../tests/test_tyre_force_contracts.py)
- diagnostics returned by `run_lap_time_simulation`

## Continue learning
- [Simulator Basics](Simulator-Basics.md)
- [Tyre Model Intro](Tyre-Model.md)
- [Tyre Model Deep Dive](Tyre-Model-Deep-Dive.md)
- [Powertrain Model and Wheel Force Flow](Powertrain-Model.md)
- [Aerodynamics Model Intro](Aero-Model.md)
- [Lessons Index](README.md)