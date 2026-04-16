import argparse
import copy
import csv
import json
import os
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from statistics import mean


SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)


def load_config(config_path):
    with open(config_path, "r") as f:
        return json.load(f)


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def level_values(nominal, pct=0.2):
    if not isinstance(nominal, (int, float)):
        raise TypeError(f"Focused sweep expects numeric parameter, got {type(nominal).__name__}")
    low = nominal * (1.0 - pct)
    high = nominal * (1.0 + pct)
    return [("low", low), ("nominal", nominal), ("high", high)]


def canonical_param(param_name):
    return {
        "aero_cp": "aero_centre_of_pressure",
    }.get(param_name, param_name)


def limiter_fractions(modes):
    if not modes:
        return {}
    counts = Counter(modes)
    total = sum(counts.values())
    if total <= 0:
        return {}
    return {k: v / total for k, v in counts.items()}


def run_suite(output_dir, include_tracks, stale_threshold, variant_names, fallback_threshold):
    from track.track import load_track
    from vehicle.vehicle import create_vehicle
    from simulator.simulator import run_lap_time_simulation

    config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config.json"))
    base_config = load_config(config_path)

    variant_map = {name: name for name in variant_names}

    track_map = {
        "FSUK": "datasets/tracks/FSUK.txt",
        "SkidpadF26": "datasets/tracks/SkidpadF26.txt",
        "StraightLineTrack": "datasets/tracks/StraightLineTrack.txt",
    }

    sweep_params = [
        "downforce_coefficient",
        "aero_cp",
        "cog_z",
        "front_track_width",
        "rear_track_width",
        "roll_stiffness",
        "max_roll_angle_deg",
        "base_mu",
    ]

    rows = []
    for track_name in include_tracks:
        track_path = track_map[track_name]
        for variant_name, variant_value in variant_map.items():
            # Create a fresh baseline vehicle to pull nominal parameter values.
            config_for_nominal = copy.deepcopy(base_config)
            config_for_nominal["track"]["file_path"] = track_path
            config_for_nominal.setdefault("ab_testing", {})["model_variant"] = variant_value
            nominal_vehicle = create_vehicle(config_for_nominal)

            for param in sweep_params:
                attr_name = canonical_param(param)
                nominal = getattr(nominal_vehicle.params, attr_name)

                for level_name, level_value in level_values(nominal, pct=0.2):
                    run_config = copy.deepcopy(config_for_nominal)
                    run_config.setdefault("ab_testing", {})["model_variant"] = variant_value
                    use_rollover_speed_cap = True
                    rollover_mode = "rollover_on"
                    run_config.setdefault("solver", {})["use_rollover_speed_cap"] = True
                    vehicle = create_vehicle(run_config)
                    setattr(vehicle.params, attr_name, level_value)

                    row = {
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                        "track": track_name,
                        "track_path": track_path,
                        "variant": variant_name,
                        "model_variant": variant_value,
                        "rollover_mode": rollover_mode,
                        "use_rollover_speed_cap": use_rollover_speed_cap,
                        "parameter": param,
                        "parameter_attr": attr_name,
                        "level": level_name,
                        "parameter_value": level_value,
                        "status": "valid",
                        "reason": "",
                        "lap_time_s": "",
                        "fallback_rate": "",
                        "forward_dominant_mode": "",
                        "backward_dominant_mode": "",
                        "forward_mode_breakdown": "",
                        "backward_mode_breakdown": "",
                        "fallback_gate_pass": "",
                    }

                    try:
                        track = load_track(track_path, run_config.get("debug_mode", False))
                        sim_result = run_lap_time_simulation(track, vehicle, run_config, display=False)
                        diagnostics = getattr(sim_result, "diagnostics", {})

                        fallback_flags = diagnostics.get("corner_fallback_used", [])
                        fallback_rate = mean([1.0 if f else 0.0 for f in fallback_flags]) if fallback_flags else 0.0

                        forward_modes = diagnostics.get("forward_limiting_mode", [])
                        backward_modes = diagnostics.get("backward_limiting_mode", [])
                        forward_breakdown = limiter_fractions(forward_modes)
                        backward_breakdown = limiter_fractions(backward_modes)

                        row["lap_time_s"] = float(sim_result.lap_time)
                        row["fallback_rate"] = float(fallback_rate)
                        row["fallback_gate_pass"] = bool(fallback_rate <= fallback_threshold)
                        row["forward_mode_breakdown"] = json.dumps(forward_breakdown, sort_keys=True)
                        row["backward_mode_breakdown"] = json.dumps(backward_breakdown, sort_keys=True)
                        row["forward_dominant_mode"] = max(forward_breakdown, key=forward_breakdown.get) if forward_breakdown else ""
                        row["backward_dominant_mode"] = max(backward_breakdown, key=backward_breakdown.get) if backward_breakdown else ""

                    except Exception as exc:
                        row["status"] = "invalid"
                        row["reason"] = f"{type(exc).__name__}: {exc}"

                    rows.append(row)

    ensure_dir(output_dir)
    runs_csv = os.path.join(output_dir, "ab_runs.csv")
    with open(runs_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    # Sensitivity summary per track x variant x parameter
    grouped = defaultdict(list)
    for r in rows:
        if r["status"] != "valid" or r["lap_time_s"] == "":
            continue
        key = (r["track"], r["variant"], r["parameter"])
        grouped[key].append(float(r["lap_time_s"]))

    sens_rows = []
    for (track_name, variant_name, parameter), lap_times in grouped.items():
        if not lap_times:
            continue
        delta = max(lap_times) - min(lap_times)
        sens_rows.append(
            {
                "track": track_name,
                "variant": variant_name,
                "parameter": parameter,
                "delta_lap_time_s": delta,
                "stale": delta < stale_threshold,
                "samples": len(lap_times),
            }
        )

    sens_csv = os.path.join(output_dir, "ab_sensitivity.csv")
    with open(sens_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["track", "variant", "parameter", "delta_lap_time_s", "stale", "samples"])
        writer.writeheader()
        writer.writerows(sens_rows)

    # Markdown summary
    invalid_count = sum(1 for r in rows if r["status"] == "invalid")
    total_count = len(rows)
    stale_counts = Counter((r["track"], r["variant"]) for r in sens_rows if r["stale"])
    valid_rows = [r for r in rows if r["status"] == "valid" and r["fallback_rate"] != ""]
    fallback_fail_counts = Counter(
        (r["track"], r["variant"])
        for r in valid_rows
        if float(r["fallback_rate"]) > float(fallback_threshold)
    )
    rollover_modes_present = sorted({r.get("rollover_mode", "") for r in rows if r.get("rollover_mode", "")})

    md_path = os.path.join(output_dir, "ab_summary.md")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# A/B Diagnostic Summary\n\n")
        f.write(f"- Generated: {datetime.now(timezone.utc).isoformat()}\n")
        f.write(f"- Total runs: {total_count}\n")
        f.write(f"- Invalid runs: {invalid_count}\n")
        f.write(f"- Tracks: {', '.join(include_tracks)}\n")
        f.write(f"- Variants: {', '.join(variant_names)}\n")
        if rollover_modes_present:
            f.write(f"- Rollover modes in this run: {', '.join(rollover_modes_present)}\n")
        f.write("- Focused parameters: downforce_coefficient, aero_cp, cog_z, front_track_width, rear_track_width, roll_stiffness, max_roll_angle_deg, base_mu\n\n")
        f.write(f"- Fallback-rate gate: pass when fallback_rate <= {fallback_threshold:.3f}\n\n")

        f.write("## Invalid Run Breakdown\n\n")
        if invalid_count == 0:
            f.write("All runs completed as valid.\n\n")
        else:
            reasons = Counter(r["reason"] for r in rows if r["status"] == "invalid")
            for reason, count in reasons.most_common():
                f.write(f"- {count}x {reason}\n")
            f.write("\n")

        f.write("## Stale Metric Count by Track/Variant\n\n")
        for track_name in include_tracks:
            for variant_name in variant_names:
                f.write(f"- {track_name} / {variant_name}: {stale_counts.get((track_name, variant_name), 0)} stale parameters\n")
        f.write("\n")

        f.write("## Fallback-Rate Gate Failures by Track/Variant\n\n")
        for track_name in include_tracks:
            for variant_name in variant_names:
                f.write(f"- {track_name} / {variant_name}: {fallback_fail_counts.get((track_name, variant_name), 0)} runs above threshold\n")
        f.write("\n")

        f.write("## Top Sensitivity Movers\n\n")
        ranked = sorted(sens_rows, key=lambda r: r["delta_lap_time_s"], reverse=True)
        for row in ranked[:12]:
            f.write(
                f"- {row['track']} / {row['variant']} / {row['parameter']}: "
                f"delta_lap_time_s={row['delta_lap_time_s']:.4f}, stale={row['stale']}\n"
            )

    return runs_csv, sens_csv, md_path


def main():
    parser = argparse.ArgumentParser(description="Run A/B diagnostic suite (Baseline vs B1) and generate CSV + Markdown outputs.")
    parser.add_argument("--output-dir", default="ab_test_outputs", help="Directory for generated artifacts")
    parser.add_argument(
        "--tracks",
        default="FSUK,SkidpadF26,StraightLineTrack",
        help="Comma-separated track set: FSUK,SkidpadF26,StraightLineTrack",
    )
    parser.add_argument(
        "--variants",
        default="baseline,b1,tyre_peak_load_clamp",
        help="Comma-separated model variants to compare",
    )
    parser.add_argument("--stale-threshold", type=float, default=0.05, help="Delta lap-time threshold (s) for stale flag")
    parser.add_argument("--fallback-threshold", type=float, default=0.15, help="Fallback-rate pass threshold")
    args = parser.parse_args()

    include_tracks = [t.strip() for t in args.tracks.split(",") if t.strip()]
    variant_names = [v.strip() for v in args.variants.split(",") if v.strip()]
    if not variant_names:
        raise ValueError("At least one variant is required")

    runs_csv, sens_csv, md_path = run_suite(
        output_dir=args.output_dir,
        include_tracks=include_tracks,
        stale_threshold=args.stale_threshold,
        variant_names=variant_names,
        fallback_threshold=args.fallback_threshold,
    )

    print("A/B suite complete")
    print(f"Runs CSV: {runs_csv}")
    print(f"Sensitivity CSV: {sens_csv}")
    print(f"Summary Markdown: {md_path}")


if __name__ == "__main__":
    main()
