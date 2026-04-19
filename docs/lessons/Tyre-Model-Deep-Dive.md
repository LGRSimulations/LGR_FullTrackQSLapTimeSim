# Tyre Model Deep Dive

## Audience
This note is for readers who already finished [Simulator Basics](Simulator-Basics.md) and [Tyre Model Intro](Tyre-Model.md).

## Goal
Give a deeper technical view of how tyre force is represented in this repository.
Show the main equations, the runtime path, and the current modelling limits.

## First principles reminder
At the contact patch, traction is distributed over area.
The net force and moment come from area integrals.

$$
\mathbf{F} = \int_A \mathbf{t}(x,y)\,dA
$$

$$
\mathbf{M}_O = \int_A \mathbf{r}(x,y) \times \mathbf{t}(x,y)\,dA
$$

For the current lap solver we mostly use in plane force channels and their resultant

$$
F_{xy} = \sqrt{F_x^2 + F_y^2}
$$

## Magic Formula context
The tyre force curves use a Pacejka style form fit to measured data.
A common pure slip shape is

$$
F(x) = D \sin\left(C \arctan\left(Bx - E\left(Bx - \arctan(Bx)\right)\right)\right)
$$

Where

- $x$ is slip input
- $D$ is force scale at current load
- $B$ controls initial slope region
- $C$ controls curve shape
- $E$ controls shoulder and peak region

This is an empirical model family.
It is fit to data and not derived from full contact patch material mechanics.

## Runtime path in this repository
Primary implementation path

- [src/vehicle/Tyres/baseTyre.py](../../src/vehicle/Tyres/baseTyre.py)

Runtime call chain

1. Parameters are loaded in [src/vehicle/vehicle.py](../../src/vehicle/vehicle.py).
2. Tyre model object is created through `create_tyre_model`.
3. Corner and speed profile routines call pure and combined tyre force methods.
4. Limits are consumed by corner equilibrium and longitudinal passes.

Related solver paths

- [src/simulator/util/vehicleDynamics.py](../../src/simulator/util/vehicleDynamics.py)
- [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py)

## Load sensitivity and base_mu
The model uses load dependent peak interpolation from TTC style datasets.
A global scale `base_mu` multiplies load derived peaks.

Conceptually

$$
D_{lat,used}(F_z) = base\_mu \cdot D_{lat,table}(F_z)
$$

$$
D_{long,used}(F_z) = base\_mu \cdot D_{long,table}(F_z)
$$

This gives a controlled way to shift force capacity while preserving curve families.

## Combined slip treatment
The current combined slip path uses a capped resultant approach.

Pure channel forces are computed first.
Then total demand is checked

$$
F_{total} = \sqrt{F_x^2 + F_y^2}
$$

Cap is built from load dependent channel peaks

$$
F_{cap} = 0.9 \cdot \sqrt{D_{lat}^2 + D_{long}^2}
$$

If demand exceeds cap, both channels are scaled by

$$
scale = \frac{F_{cap}}{F_{total}}
$$

$$
F_x' = scale\,F_x, \qquad F_y' = scale\,F_y
$$

This preserves vector direction and enforces a consistent budget.
It is a practical approximation and not a full MF combined slip weighting model.

## Worked numeric example
Assume

- pure $F_y = 3800$ N
- pure $F_x = 3200$ N
- $D_{lat} = 4000$ N
- $D_{long} = 3200$ N

Cap

$$
F_{cap} = 0.9 \cdot \sqrt{4000^2 + 3200^2} \approx 4610\,N
$$

Demand

$$
F_{total} = \sqrt{3800^2 + 3200^2} \approx 4964\,N
$$

Scale

$$
scale = \frac{4610}{4964} \approx 0.929
$$

Returned

- $F_x' \approx 2973$ N
- $F_y' \approx 3530$ N

## Effective loaded radius note
Slip and kinematic relations in literature use effective rolling radius.
For free rolling relation

$$
V_x \approx r_e\Omega
$$

So if `wheel_radius` does not represent a loaded effective value, speed to RPM and slip related computations can drift.
For this project, treating wheel radius as loaded effective radius is the healthier assumption.

## Validation and guardrails
Relevant tests and reports

- [tests/test_tyre_force_contracts.py](../../tests/test_tyre_force_contracts.py)
- [tests/test_tyre_validity_domain_contracts.py](../../tests/test_tyre_validity_domain_contracts.py)
- [tests/test_limiting_case_contracts.py](../../tests/test_limiting_case_contracts.py)
- [tools/analysis/compare_tyre_model.py](../../tools/analysis/compare_tyre_model.py)

Typical checks include

- zero and limiting behavior
- out of domain counters
- high load growth behavior
- sensitivity gates on representative tracks

## Known limits
Current model does not include full transient tyre state evolution.
Examples

- relaxation length dynamics
- full temperature and pressure state evolution
- full aligning moment path in this compact workflow
- full carcass dynamics

These are deliberate tradeoffs for speed and robustness in lap sweeps.

## References
- Pacejka, H. B. Tire and Vehicle Dynamics
- Bakker, Nyborg, Pacejka 1987 SAE tyre modelling paper https://www.theoryinpracticeengineering.com/resources/tires/pacejka87.pdf
- MathWorks Tire Road Interaction Magic Formula https://www.mathworks.com/help/sdl/ref/tireroadinteractionmagicformula.html
- x-engineer longitudinal Magic Formula summary https://x-engineer.org/automotive-engineering/chassis/vehicle-dynamics/tire-model-for-longitudinal-forces/
