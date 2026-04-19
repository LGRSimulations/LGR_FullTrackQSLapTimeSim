# Tyre Model and Pacejka Flow

## Audience
This note is for developers who need a low-level understanding of the tyre model.

## Goal
Explain what first principles says about tyre force, what Magic Formula means in literature, and how this repository computes and applies tyre forces in code.

This page uses static diagram images so it renders in any Markdown viewer.

## Quick Answer to the First-Principles Question
Your statement is mostly correct, with one important extension.

- At the contact patch, the tyre-road interaction is a distributed stress field over area.
- The physically meaningful net output is the integrated resultant force and resultant moment.
- In vehicle dynamics, we resolve that net force into components Fx, Fy, and Fz because the equations of motion are axis based.
- Fx and Fy are not separate physical inventions. They are component views of the same physical resultant.

So the right wording is

- First principles gives a resultant force and moment from contact patch stresses.
- Practical simulation resolves the resultant into Fx and Fy to compute vehicle motion.

## First Principles View
At contact patch level, the tyre transmits distributed tractions.
If $A$ is contact patch area and $\mathbf{t}(x,y)$ is local traction vector, then

$$
\mathbf{F} = \int_A \mathbf{t}(x,y)\,dA
$$

and the net moment about point $O$ is

$$
\mathbf{M}_O = \int_A \mathbf{r}(x,y) \times \mathbf{t}(x,y)\,dA
$$

For lap simulation, we usually need in-plane force components and their magnitude

$$
F_{xy} = \sqrt{F_x^2 + F_y^2}
$$

This is why literature commonly reports tyre behavior in terms of $F_x(\kappa, \alpha, F_z, \gamma)$, $F_y(\kappa, \alpha, F_z, \gamma)$, and often $M_z$.

### Force Shape In One View
![Tyre force shape flow](figures/tyre_force_shape_flow.png)

Text fallback
```
Slip input -> Pacejka style curve -> Tyre force output
Normal load -> Load lookup table -> Peak force scale D -> Pacejka style curve
```

## Where Pacejka Fits in Literature
The Magic Formula is an empirical or semi-empirical model.
It is fit to measured tyre data rather than derived from full contact patch material mechanics.

That is standard practice in vehicle dynamics because it gives good fidelity per compute cost for handling and lap simulation.

A common pure-slip form is

$$
F(x) = D \sin\left(C \arctan\left(Bx - E\left(Bx - \arctan(Bx)\right)\right)\right)
$$

Where

- $x$ is slip input
- $D$ is peak scale
- $B$ is stiffness shaping
- $C$ is curve shape
- $E$ is peak shoulder curvature shaping

In this repository, the model uses TTC-derived data to set load-sensitive peak scaling and fits global shape parameters for lateral and longitudinal channels.

### Curve Meaning
![Tyre curve meaning](figures/tyre_curve_meaning.png)

Text fallback
```
Low slip -> Force rises quickly -> Near peak -> Force saturates -> Combined slip limit
```

## Resultant Force and Component Forces
The contact patch does produce one net force vector, and in-plane magnitude matters.

$$
F_{resultant} = \sqrt{F_x^2 + F_y^2}
$$

Vehicle models still use $F_x$ and $F_y$ explicitly because chassis equations require directional components for

- force balance
- yaw moment balance
- acceleration integration

So both views are needed and both are physically valid.

### Resultant View
![Tyre resultant force vector](figures/tyre_resultant_force.png)

Text fallback
```
Pure Fx + Pure Fy -> Resultant force budget -> Combined slip scaling -> Final tyre force pair
```

## Combined Slip in Literature vs This Model
Literature uses several combined-slip approaches.
Two common families are

- friction circle or ellipse constraints
- Magic Formula combined-slip weighting functions

This code uses a friction-circle style magnitude cap built from load-dependent pure-channel peaks.

In [src/vehicle/Tyres/baseTyre.py](../../src/vehicle/Tyres/baseTyre.py), the combined path computes

$$
F_{total} = \sqrt{F_x^2 + F_y^2}
$$

$$
F_{cap} = 0.9 \cdot \sqrt{D_{lat}^2 + D_{long}^2}
$$

If $F_{total} > F_{cap}$, both channels are scaled by

$$
scale = \frac{F_{cap}}{F_{total}}, \quad F_x' = scale\,F_x, \quad F_y' = scale\,F_y
$$

This preserves force direction while enforcing a combined budget.
It is a pragmatic approximation, not a full MF combined-slip law.

## Worked Example with Resultant Force

### Real-world intuition
On corner exit, the tyre is asked for both lateral and drive force.

Assume pure channel demands

- $F_y = 3800$ N
- $F_x = 2600$ N

Then

$$
F_{resultant} = \sqrt{3800^2 + 2600^2} \approx 4604\,\text{N}
$$

If available combined grip is below this demand, the tyre must return a reduced pair.

### This model path
The combined-force routine in [src/vehicle/Tyres/baseTyre.py](../../src/vehicle/Tyres/baseTyre.py)

1. Computes pure $F_y$ from slip angle and normal load.
2. Computes pure $F_x$ from slip ratio and normal load.
3. Computes load-based peak scales $D_{lat}$ and $D_{long}$.
4. Computes $F_{cap}$ and scales if needed.

### Numeric example in code terms
Assume

- $D_{lat} = 4000$ N
- $D_{long} = 3200$ N

Then

$$
F_{cap} = 0.9 \cdot \sqrt{4000^2 + 3200^2} \approx 4610\,\text{N}
$$

With $F_x = 2600$ N and $F_y = 3800$ N, demand is about $4604$ N, so no scaling.

If pure $F_x$ rises to $3200$ N with $F_y = 3800$ N

$$
F_{total} = \sqrt{3800^2 + 3200^2} \approx 4964\,\text{N}
$$

Now cap is exceeded

$$
scale = \frac{4610}{4964} \approx 0.929
$$

Returned forces are about

- $F_x' \approx 2973$ N
- $F_y' \approx 3530$ N

Reference for verification workflow style

- [tools/analysis/compare_tyre_model.py](../../tools/analysis/compare_tyre_model.py)

## How This Repository Builds Tyre Force
Runtime path

1. [src/vehicle/vehicle.py](../../src/vehicle/vehicle.py) loads parameters and constructs the tyre model.
2. [src/vehicle/Tyres/baseTyre.py](../../src/vehicle/Tyres/baseTyre.py) provides pure and combined force methods.
3. [src/simulator/util/vehicleDynamics.py](../../src/simulator/util/vehicleDynamics.py) uses tyre forces in corner equilibrium.
4. [src/simulator/util/calcSpeedProfile.py](../../src/simulator/util/calcSpeedProfile.py) uses tyre limits for speed profile bounds.

### Runtime Flow
![Tyre runtime flow](figures/tyre_runtime_flow.png)

Text fallback
```
parameters json -> vehicle.py -> LookupTableTyreModel
LookupTableTyreModel -> vehicleDynamics corner solve -> Lap time result
LookupTableTyreModel -> calcSpeedProfile speed cap -> Lap time result
```

## What the Lookup Tables Provide
The lookup tables make peak force load sensitive.
That is the key step beyond fixed-$\mu$ assumptions.

Current behavior

- Lateral peak scale comes from normal-load versus lateral-force data.
- Longitudinal peak scale comes from normal-load versus longitudinal-force data.
- Longitudinal slip units are auto-detected as ratio-like or percent-like.

## Validation and Guardrails
Related checks and history

- [docs/MAJOR_CHANGE_LOG.md](../../docs/MAJOR_CHANGE_LOG.md)
- [docs/PARAMETER_ENFORCEMENT_AUDIT_ROADMAP.md](../../docs/PARAMETER_ENFORCEMENT_AUDIT_ROADMAP.md)
- [tools/analysis/compare_tyre_model.py](../../tools/analysis/compare_tyre_model.py)
- [tests/test_tyre_force_contracts.py](../../tests/test_tyre_force_contracts.py)
- [tests/test_limiting_case_contracts.py](../../tests/test_limiting_case_contracts.py)
- [tests/test_tyre_validity_domain_contracts.py](../../tests/test_tyre_validity_domain_contracts.py)

Current guardrails include

- validity-domain diagnostics
- out-of-domain counters
- high-load clamp variant for extrapolation control
- A/B verification for growth behavior

## Known Limits Relative to Full Tyre Literature
This model is not a full transient thermo-mechanical tyre model.
It does not currently model

- relaxation length dynamics
- explicit aligning moment $M_z$ output in this simplified path
- full temperature and pressure state evolution
- detailed carcass and belt dynamics
- full MF combined-slip weighting formulations

The tradeoff is deliberate.
The benefit is stable and fast behavior for lap-time simulation sweeps.

## Developer Checklist
Before changing tyre logic, check

1. Do we still preserve load-to-peak behavior with normal load
2. Do we still preserve a defensible combined-slip envelope
3. Did the factory path stay aligned with the production model
4. Did verification docs and contracts get updated when force envelopes changed

## References for Further Reading
- Pacejka, H. B. Tire and Vehicle Dynamics
- Bakker, Nyborg, Pacejka 1987 SAE tyre modeling paper https://www.theoryinpracticeengineering.com/resources/tires/pacejka87.pdf
- MathWorks Tire-Road Interaction Magic Formula documentation https://www.mathworks.com/help/sdl/ref/tireroadinteractionmagicformula.html
- x-engineer longitudinal Magic Formula summary https://x-engineer.org/automotive-engineering/chassis/vehicle-dynamics/tire-model-for-longitudinal-forces/

## Related Lessons
- [Lessons Index](README.md)
- [How We Added Rollover Constraints and Made It Realistic](../how we added rollover constraints and made it realistic.md)