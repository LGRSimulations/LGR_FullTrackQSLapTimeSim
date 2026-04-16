# Major Change Log

Purpose: standardised record of major simulator and tyre-model changes.

Usage rules:
- Log every major change with the exact section format below.
- Include UTC timestamp for each entry.
- Explain the physical reasoning and engineering trade-offs in plain language.
- Keep entries focused on measurable engineering outcomes.
- Include pass/fail gate results for invariants, RMSE, and fallback-rate thresholds.

---

Timestamp: 2026-04-16T18:20:00Z
Owner: Copilot + Project Team
Change ID: tyre-peak-load-clamp-v1
Scope: tyre force envelope realism, validation gating, A/B comparison clarity

## 1) Problem

### High-Level
- The simulator could become too optimistic in high-load conditions, reporting grip and g-levels that were higher than we could justify from measured tyre data.
- This reduced trust in rollover conclusions and in lap-time comparisons, because the model could look fast for the wrong reason.

### Low-Level
- The tyre model used a curve that continued beyond the last measured normal-load point.
- In practice, this meant that once the virtual tyre entered a higher load region than the dataset covered, predicted peak force could keep increasing without evidence.
- From first principles, we should not claim additional grip where we have no measured support.

## 2) Diagnosis
- Evidence used:
	- Tyre verification plots and error metrics.
	- Rollover and G-G diagnostics where high combined g appeared.
	- A/B outputs tracking fallback behavior and sensitivity patterns.
- Root cause:
	- Peak-force prediction above measured load range was being extrapolated, which can overstate available grip.
- Alternatives ruled out:
	- Basic tyre invariants remained valid, so the issue was not zero-load behavior.
	- RMSE within measured load bins was already good, so the core in-range fit was not the primary problem.

## 3) Solution and Implementation
- Engineering intent:
	- Constrain the tyre model to a defensible force envelope when loads go beyond measured TTC data.
- What changed:
	- Added a controlled variant that caps high-load peak-force lookup at the highest measured load.
	- Kept baseline behavior available to preserve comparison continuity.
	- Added explicit gates so pass/fail is visible for tyre behavior and fallback usage.
- Why this is physically reasonable:
	- Tyre grip is strongly load-dependent, but not unconstrained.
	- If measurement coverage ends, assuming continued linear growth is unsafe.
	- Clamping at the measured boundary is conservative and better aligned with engineering evidence.
- Verification added:
	- Focused contract checks for high-load growth behavior.
	- High-load growth ratio gate in tyre verification.
	- Fallback-rate gate visibility in A/B summaries.

## 4) Impact and Explanation
- Physics correctness impact:
	- Reduced risk of non-physical force inflation in high-load operating points.
- Lap-time trustworthiness impact:
	- Lower chance of optimistic lap-time deltas caused by unsupported grip assumptions.
- Diagnostics stability impact:
	- Clearer pass/fail interpretation across tyre verification and A/B outputs.
- Limitations and next step:
	- This does not yet solve combined-slip budgeting end-to-end in the speed-profile passes.
	- Next priority is to tighten corner fallback behavior so solver fallbacks reflect physically bounded states.

## Validation Gates (Required)
- Tyre invariants: PASS
- RMSE thresholds: PASS
- Fallback-rate thresholds: PASS (smoke suite baseline vs tyre_peak_load_clamp)

## Reproducibility Notes (Optional)
- uv run python -m unittest tests.test_tyre_force_contracts tests.test_tyre_peak_load_clamp_contracts
- uv run python tools/analysis/compare_tyre_model.py --validate --model-variant baseline --rmse-threshold-pct 12 --max-high-load-growth-ratio 1.35
- uv run python tools/analysis/compare_tyre_model.py --validate --model-variant tyre_peak_load_clamp --rmse-threshold-pct 12 --max-high-load-growth-ratio 1.05
- uv run python src/ab_testing/run_ab_suite.py --tracks StraightLineTrack --variants baseline,tyre_peak_load_clamp --fallback-threshold 0.15 --output-dir ab_test_outputs/smoke_tyre_clamp

---

Timestamp: 2026-04-16T19:05:00Z
Owner: Copilot + Project Team
Change ID: corner-solver-retry-tiers-v1
Scope: corner equilibrium robustness, fallback observability, per-circuit solve diagnostics

## 1) Problem

### High-Level
- When corner equilibrium failed, the simulator dropped straight to fallback speed with limited recovery attempts.
- This made it harder to distinguish true physical limits from numerical solve fragility.

### Low-Level
- The corner solver path used a single solve attempt per point and then fallback.
- There was no warm-start continuation between adjacent points and limited visibility into how often retries were needed.

## 2) Diagnosis
- Evidence used:
	- Existing fallback-rate diagnostics from lap simulation outputs.
	- Circuit runs on FSUK, SkidpadF26, and StraightLineTrack.
- Root cause:
	- Some FSUK high-curvature segments remained sensitive to initial guess and bounds; one-shot solves were not robust enough.
- Alternatives ruled out:
	- This was not a global solver collapse: Skidpad and StraightLine solved without fallback in baseline runs.

## 3) Solution and Implementation
- Engineering intent:
	- Prefer physically valid equilibrium solutions through structured retries before fallback.
- What changed:
	- Added retry tiers: base solve, warm-start solve, conservative-bound solve.
	- Added optional warm-start and speed upper-bound inputs to the corner state solver.
	- Added diagnostics channels for retry count, selected solve tier, and per-point physical cap speed.
- Why this is physically reasonable:
	- Adjacent track points are dynamically similar, so warm starts should improve numerical convergence without changing governing physics.
	- Conservative upper bounds avoid spending solve effort in unlikely speed regions.
- Verification added:
	- Existing solver and tyre contract tests run as non-regression checks.
	- Circuit-level fallback/retry telemetry collected directly from simulation diagnostics.

## 4) Impact and Explanation
- Physics correctness impact:
	- No new force model assumptions were introduced; this is a solve-robustness improvement.
- Lap-time trustworthiness impact:
	- Reduces unnecessary fallback usage where a physically valid equilibrium is recoverable by better initialization.
- Diagnostics stability impact:
	- Clear visibility into whether a point solved via base tier, warm start, conservative tier, or fallback.
- Limitations and next step:
	- FSUK still shows localized fallback (~1.55% in baseline), so next work is a constrained fallback replacement and failure-reason taxonomy.

## Validation Gates (Required)
- Tyre invariants: PASS
- RMSE thresholds: PASS (unchanged, no tyre-curve edits in this step)
- Fallback-rate thresholds:
	- FSUK: PASS (0.0155 with threshold 0.15 absolute fraction = 15%)
	- SkidpadF26: PASS (0.0000)
	- StraightLineTrack: PASS (0.0000)

## Reproducibility Notes (Optional)
- uv run python -m unittest tests.test_solver_contracts tests.test_tyre_force_contracts tests.test_tyre_peak_load_clamp_contracts
- uv run python -c "import sys,copy,json,collections; from statistics import mean; sys.path.insert(0,'src'); from track.track import load_track; from vehicle.vehicle import create_vehicle; from simulator.simulator import run_lap_time_simulation; cfg=json.load(open('config.json')); tracks=['datasets/tracks/FSUK.txt','datasets/tracks/SkidpadF26.txt','datasets/tracks/StraightLineTrack.txt']; print('track,fallback_rate,retry_mean,retry_max,top_methods');\nfor t in tracks: c=copy.deepcopy(cfg); c['track']={'file_path':t}; v=create_vehicle(c); tr=load_track(t,c.get('debug_mode',False)); r=run_lap_time_simulation(tr,v,c,display=False); fb=r.diagnostics.get('corner_fallback_used',[]); rc=r.diagnostics.get('corner_retry_count',[]); sm=r.diagnostics.get('corner_solve_method',[]); fr=mean([1.0 if x else 0.0 for x in fb]) if fb else 0.0; rm=mean(rc) if rc else 0.0; rmax=max(rc) if rc else 0; cnt=collections.Counter(sm); top=';'.join([f'{k}:{v}' for k,v in cnt.most_common(3)]); print(f'{t},{fr:.4f},{rm:.3f},{rmax},{top}')"

---

Timestamp: 2026-04-16T19:45:00Z
Owner: Copilot + Project Team
Change ID: fallback-rootcause-and-rollover-visibility-v1
Scope: fallback physical constraints, solver failure attribution, A/B rollover-mode traceability

## 1) Problem

### High-Level
- We still saw optimistic combined-g behavior, but diagnostics did not clearly separate physical-limit behavior from solver failure behavior.
- We also had ambiguity when comparing fallback counts across runs because rollover mode was not explicit in A/B outputs.

### Low-Level
- Fallback speed selection was still largely formula-based and did not include a continuity cap from the previous solved point.
- Solver failure reasons were not carried through clearly in diagnostics, so root-cause localization was hard.
- A/B output rows did not include explicit `rollover_on` / `rollover_off` context.

## 2) Diagnosis
- Evidence used:
	- FSUK rollover diagnostics with high combined-g peaks.
	- Direct one-shot vs retry-tier comparisons on FSUK.
	- Inspection of per-point solver outcomes and fallback locations.
- Root cause:
	- Remaining fallback points were localized solve-feasibility issues (residual/state-bound) rather than global model failure.
	- Mode-mixing (rollover on vs off) explained previous confusion around fallback counts.
- Alternatives ruled out:
	- Tyre invariant and contract failures were not the driver; contracts remained passing.

## 3) Solution and Implementation
- Engineering intent:
	- Make fallback conservative and physically bounded, and make solver failures diagnostically actionable.
- What changed:
	- Added constrained fallback speed selection using physical cap and continuity cap.
	- Added explicit failure reason tagging from corner solver (`root_failed`, `residual_not_met`, `state_bounds_exceeded`, etc.).
	- Added tier-level failure traces in diagnostics.
	- Added explicit rollover context columns to A/B runs: `rollover_mode`, `use_rollover_speed_cap`.
- Why this is physically reasonable:
	- Fallback should not create sudden optimistic speed jumps relative to neighboring points.
	- Failure diagnostics should identify whether rejection came from physics-consistency checks or numerical solve failure.
- Verification added:
	- Contract suite rerun (solver + tyre invariants + tyre clamp contracts).
	- FSUK failure reason distribution extracted from diagnostics.
	- A/B quick run confirms rollover mode fields are emitted in CSV/summary.

## 4) Impact and Explanation
- Physics correctness impact:
	- Fallback path is now more conservative and less likely to inject non-physical spikes.
- Lap-time trustworthiness impact:
	- Better confidence that unresolved points are capped consistently rather than inflated by unconstrained fallback.
- Diagnostics stability impact:
	- Root-cause attribution is now visible: FSUK baseline run showed 254 points with no issue, and only 4 fallback points split across residual/state-bound categories.
- Limitations and next step:
	- Combined-g values are still high; remaining issue is not just fallback, but force budgeting in forward/backward propagation.

## Validation Gates (Required)
- Tyre invariants: PASS
- RMSE thresholds: PASS (no tyre fit changes in this step)
- Fallback-rate thresholds:
	- FSUK (rollover_on): PASS under current gate definition (`fallback_rate <= 0.15` absolute fraction)
	- SkidpadF26: PASS
	- StraightLineTrack: PASS

## Reproducibility Notes (Optional)
- uv run python -m unittest tests.test_solver_contracts tests.test_tyre_force_contracts tests.test_tyre_peak_load_clamp_contracts
- uv run python src/diagnostics/gg_rollover_suite.py --track datasets/tracks/FSUK.txt --output-dir artifacts/diagnostics/gg_rollover_suite
- uv run python -c "import sys,json; sys.path.insert(0,'src'); from track.track import load_track; from vehicle.vehicle import create_vehicle; from simulator.simulator import run_lap_time_simulation; from collections import Counter; cfg=json.load(open('config.json')); cfg['track']={'file_path':'datasets/tracks/FSUK.txt'}; v=create_vehicle(cfg); tr=load_track('datasets/tracks/FSUK.txt',cfg.get('debug_mode',False)); res=run_lap_time_simulation(tr,v,cfg,display=False); print(dict(Counter(res.diagnostics.get('corner_failure_reason',[]))))"
- uv run python src/ab_testing/run_ab_suite.py --tracks StraightLineTrack --variants baseline --fallback-threshold 0.15 --output-dir ab_test_outputs/smoke_rollover_flag_quick

---

Timestamp: 2026-04-16T20:05:00Z
Owner: Copilot + Project Team
Change ID: combined-force-budgeting-propagation-v1
Scope: forward/backward speed propagation realism under combined lateral + longitudinal demand

## 1) Problem

### High-Level
- Combined-g values were still too optimistic even after fallback and retry-tier improvements.
- This suggested that solver robustness alone was not enough; force budgeting in propagation was still too generous.

### Low-Level
- Forward and backward passes used strong pure longitudinal capacity while lateral demand was present.
- Without consistent friction-ellipse budgeting through propagation, the model could overstate usable longitudinal authority in corners.

## 2) Diagnosis
- Evidence used:
	- FSUK rollover diagnostics showing high combined-g values.
	- Inspection of forward/backward force-cap logic in speed propagation path.
- Root cause:
	- Longitudinal force caps were not consistently reduced by lateral demand in both propagation passes.
- Alternatives ruled out:
	- Remaining fallback points were localized and low-count, so they were not the only source of high combined-g.

## 3) Solution and Implementation
- Engineering intent:
	- Enforce a consistent combined-force budget so acceleration/braking authority reduces as lateral demand rises.
- What changed:
	- Added total pure-slip force-cap helper for longitudinal and lateral channels.
	- Added friction-ellipse budget scale from lateral demand ratio.
	- Applied that scale to longitudinal traction in forward pass and braking authority in backward pass.
	- Exposed new diagnostics channels for lateral caps and combined-budget scales.
- Why this is physically reasonable:
	- Tyres cannot deliver independent peak Fx and peak Fy simultaneously.
	- A friction-ellipse style budget enforces this coupling in a physically interpretable way.
- Verification added:
	- Existing contract suite rerun.
	- FSUK rollover diagnostics rerun after propagation update.

## 4) Impact and Explanation
- Physics correctness impact:
	- Improves consistency between tyre force limits and speed-profile propagation under combined loading.
- Lap-time trustworthiness impact:
	- Reduces optimistic acceleration/braking in cornered states.
- Diagnostics stability impact:
	- Added telemetry to track when and how longitudinal authority was reduced by lateral demand.
- Limitations and next step:
	- Current representative lateral slip-angle cap is still a simplified proxy.
	- Next step is parameter-activation audit and per-parameter influence checks to ensure all root JSON parameters affect the expected physics pathways.

## Validation Gates (Required)
- Tyre invariants: PASS
- RMSE thresholds: PASS (no tyre fit changes in this step)
- Fallback-rate thresholds:
	- FSUK (rollover_on): PASS (0.0155 under current gate definition)
	- SkidpadF26: PASS (from previous baseline sweep)
	- StraightLineTrack: PASS (from previous baseline sweep)

## Reproducibility Notes (Optional)
- uv run python -m unittest tests.test_solver_contracts tests.test_tyre_force_contracts tests.test_tyre_peak_load_clamp_contracts
- uv run python src/diagnostics/gg_rollover_suite.py --track datasets/tracks/FSUK.txt --output-dir artifacts/diagnostics/gg_rollover_suite
