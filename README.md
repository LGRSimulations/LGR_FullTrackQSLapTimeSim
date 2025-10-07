
# 🏎️ Lap Time Simulator Development Roadmap

## Purpose of this repo
- Validate design decisions, and test certain parameters of LGR FS25/26 car
- Explore the breadth and depth of Lap Time Sims


This repository outlines a long-term roadmap for building a modular, high-performance **lap time simulation framework**. The end goal is to develop a simulation and machine learning system capable of:

- Recommending optimal **vehicle setups**
- Determining the **optimal racing line**
- Working across **different vehicle models** (ICE, EV, Hybrid)
- Generating data for validation, component design

---

## 🔁 Overview of Phases

| Phase | Focus | Purpose |
|-------|-------|---------|
| 1️⃣ | Point Mass Sim | Build fundamental lap time sim and racing line optimizer |
| 2️⃣ | Dual-Axle Bicycle Model | Add load transfer, tyre force realism, yaw dynamics |
| 3️⃣ | Setup-Aware Sim | Output full telemetry, evaluate setups, generate component loads |

---

## Phase 1 – 🧠 Point Mass Lap Time Simulator

### 🎯 Goal
- Compute **optimal velocity profile** and **racing line**
- Use **GGV (acceleration envelope)** or μ-limited lookup tables

### 🧰 Features
- Track as curvature or x/y map
- Forward-backward velocity integration
- Simple tyre model or interpolated μ from lookup table
- Optional aerodynamic drag/downforce

### ✅ Validation
- Circular track test → verify lateral limit: `a_lat = μ·g`
- Straight-line acceleration vs power curve
- Analytical results (e.g., Milliken ideal line)

### 🔍 Insights
- Max performance envelopes
- Line vs speed trade-offs
- Lap time sensitivity to grip/aero

### ⚡️ Performance
- Millisecond-scale simulation
- Suitable for parameter sweeps & ML pretraining

---

## Phase 2 – 🚗 Dual-Axle Bicycle Model Simulator

### 🎯 Goal
- Introduce **yaw, understeer/oversteer**, and **realistic tyre forces**
- Enable study of **brake bias, load transfer, weight distribution**

### 🧰 Features
- Front/rear axle forces
- Longitudinal & lateral load transfer
- Aero CoP influence
- tyre load sensitivity (from lookup table or Pacejka)
- Steering & throttle control logic

### ✅ Validation
- Step steer: yaw rate vs steer angle
- Understeer gradient analysis
- Braking & cornering overlap (friction ellipse)

### 🔍 Insights
- Influence of CoG height, brake bias, and weight distribution
- More accurate lap time prediction than point-mass
- Lateral load distribution under G

### ⚡️ Performance
- Still fast (<100ms/lap)
- Useful for tuning & basic optimization loops

---

## Phase 3 – 🔧 Setup-Aware Lap Time Simulator

### 🎯 Goal
- Simulate **vehicle setup tuning**
- Output **component-level loads** (pushrods, dampers, uprights)
- Full telemetry-style outputs (acceleration, pedals, velocities)

### 🧰 Features
- Vehicle config parameters: toe, camber, spring rates, ARB, ride height
- Pushrod load calculation from body accelerations
- Pedal/brake/throttle mapping to forces
- Support for ICE and EV drivetrains

### ✅ Validation
- Compare to real-world data or IPG CarMaker outputs
- Structural force checks vs FEA expectations
- Component-level load tracking for design input

### 🔍 Insights
- Setup sensitivity for performance & durability
- Loads for FEA of uprights, wishbones, mounts
- Realistic telemetry for ML training/validation

### ⚡️ Performance
- ~0.5–1s per lap depending on complexity
- Still lightweight enough for use in looped optimization

---

## Phase 4 – 🤖 Reinforcement Learning Integration

### 🎯 Goal
- Use the sim as an **RL environment** for:
  - Setup recommendation
  - Optimal line control
  - Fast policy transfer across vehicle models

### 🧰 Features
- Observation space: state, track, setup
- Action space: setup parameters, line adjustments, pedal inputs
- Reward: lap time, grip usage, energy consumption

### ✅ Validation
- Compare RL performance to sim-optimal baseline
- Analyze learned setup vs engineering best practices
- Visualize GGV usage and line smoothness

### 🔍 Outcomes
- Autonomous performance engineering agent
- Generalized tuning across vehicle platforms
- Fast evaluation for new tracks or conditions

---

## 💻 Performance Considerations

- Vectorized Python using `numpy`
- Use `scipy.interpolate` for fast tyre model lookup
- Profile & optimize bottlenecks (`cProfile`, `line_profiler`)
- For future acceleration: consider `numba` or `cython`

---

## 🔧 Tools and Libraries

| Purpose | Tool |
|--------|------|
| Sim backend | Python (NumPy, SciPy) |
| tyre models | Interpolated lookup, Magic Formula |
| Data handling | Pandas, HDF5/Parquet |
| Plotting | Matplotlib, Seaborn |
| RL | Stable Baselines3, PyTorch |
| UI (Optional) | Streamlit, Dash |

---

## 📦 Want to Contribute?

Start with:
- `point_mass_sim.py`: Base sim with GGV limits
- `track_loader.py`: Load & process track curvature or x/y data
- `tyre_model.py`: Interpolation wrapper for tyre lookup tables
- `setup_optimization.py`: Framework to sweep configs for Phase 3

---

## 📬 Questions?

Open an issue or discussion! Whether you're working on EV strategy, motorsport simulation, or ML for vehicle dynamics — let's build something great.

