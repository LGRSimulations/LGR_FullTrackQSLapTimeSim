# Milestone 1: Equations and Units Audit

Purpose: establish a single source of truth for governing equations, units, and sign conventions used by the current quasi-static solver path.

Status: in progress

## 1) Sign Conventions

- Curvature `K` [1/m]: sign indicates turn direction; magnitudes are used in cap logic where direction is irrelevant.
- Lateral tyre forces `Fy_f`, `Fy_r` [N]: positive values follow current tyre model sign convention.
- Steering angle `delta` [rad internal, deg at API boundaries].
- Sideslip angle `beta` [rad internal, deg at API boundaries].

## 2) Governing Equations

### Corner equilibrium solver

Implemented in `find_vehicle_state_at_point`.

- Lateral force balance:

$$
m v^2 K = F_{y,f} + F_{y,r}
$$

- Yaw moment balance about CoG:

$$
a F_{y,f} = b F_{y,r}
$$

- Slip-angle relations (small-angle bicycle form):

$$
\alpha_f = \delta - \beta - aK
$$

$$
\alpha_r = -\beta + bK
$$

### Rollover speed bound

Used as a hard upper bound in corner solving and speed-cap composition.

$$
v_{roll} = \sqrt{\frac{t}{2h} g R}, \quad R = \frac{1}{|K|}
$$

### Forward-pass speed propagation

$$
v_{i,pred}^2 = v_{i-1}^2 + 2 a_{x,i} \Delta s
$$

with

$$
a_{x,i} = \frac{F_{x,i}}{m}, \quad F_{x,i} = F_{traction,i} - F_{drag,i}
$$

### Backward-pass braking propagation

$$
v_{i,brake}^2 = v_{i+1}^2 + 2 a_{brake,i} \Delta s
$$

with friction-budgeted braking and drag contribution.

## 3) Units Audit Table

- `m` mass: kg
- `v` speed: m/s
- `K` curvature: 1/m
- `R` radius: m
- `F` forces: N
- `M` moments: N m
- `a` accelerations: m/s^2
- `delta`, `beta`, `alpha`: rad internal, deg at tyre API boundary
- `t`, `h`, `a`, `b`, `L`, `ds`: m

Audit rule: each additive equation must sum same dimensions on both sides.

## 4) Current Limiting-Case Contracts

The following contracts are now covered by dedicated tests:

- Near-zero curvature returns straight-cap speed and zero steer/sideslip.
- Longitudinal combined-slip budget scale remains bounded in [0, 1] with expected endpoints.
- Zero powertrain torque does not create forward acceleration on a straight segment.
- Zero-speed backward propagation remains stationary.

## 5) Open Follow-Ups

- Extend sign-convention coverage to lateral direction consistency checks at left/right curvature pairs.
- Add explicit dimensional assertion helpers for equation terms in tests.
- Add per-equation residual channels to diagnostics for easier post-run audits.