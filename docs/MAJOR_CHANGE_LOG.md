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
