#!/usr/bin/env python3
"""
Automated KF Variance Tuner using Optuna (v2)
===============================================
Optimizes the estimator noise variance parameters by:
1. Writing trial parameters to proj2.yaml
2. Launching the simulation headless via bash (to properly source ROS2)
3. Recording a rosbag (MCAP)
4. Computing RMSE of estimated vs ground truth pose
5. Returning the RMSE as the objective to minimize

Dependencies:
    pip install optuna rosbags numpy pyyaml

Usage:
    python kf_variance_tuner.py [--n-trials 50] [--sim-duration 60] [--workspace ~/ee4308]
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

try:
    import optuna
except ImportError:
    sys.exit("ERROR: optuna not installed. Run: pip install optuna")

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import get_typestore, Stores
except ImportError:
    sys.exit("ERROR: rosbags not installed. Run: pip install rosbags")


# ===========================================================================
# Configuration
# ===========================================================================
PARAMS_REL_PATH = "src/ee4308_bringup/params/proj2.yaml"
RECORD_TOPICS = ["/drone/odom", "/drone/true_odom"]
WARMUP_SECONDS = 5.0


# ===========================================================================
# Helpers
# ===========================================================================
def resolve_field(msg, field_path: str):
    obj = msg
    for attr in field_path.split("."):
        obj = getattr(obj, attr)
    return obj


def get_stamp_sec(msg) -> float:
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        s = msg.header.stamp
        return s.sec + s.nanosec * 1e-9
    return None


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def limit_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def kill_ros_processes():
    """Aggressively kill any leftover ROS/Gazebo processes between trials."""
    kill_targets = [
        "gz sim", "gz-sim", "ruby", "gzserver", "gzclient",
        "component_container", "robot_state_publisher",
        "rviz2", "parameter_bridge",
    ]
    for target in kill_targets:
        subprocess.run(
            ["pkill", "-9", "-f", target],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    subprocess.run(["pkill", "-9", "ruby"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "gz"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)


def update_yaml_params(yaml_path: Path, params: dict):
    """Update the estimator variance parameters in proj2.yaml."""
    with open(yaml_path, "r") as f:
        config = yaml.safe_load(f)

    est_params = config
    for key in ["drone", "estimator", "ros__parameters"]:
        if key not in est_params:
            est_params[key] = {}
        est_params = est_params[key]

    for k, v in params.items():
        est_params[k] = float(v)

    with open(yaml_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)


# ===========================================================================
# Sim launch + record
# ===========================================================================
def launch_sim_and_record(
    workspace: Path,
    bag_dir: Path,
    sim_duration: float,
    startup_wait: float = 12.0,
) -> bool:
    """
    Launch sim and record using subprocess.Popen, inheriting the current
    shell's environment. This works correctly inside distrobox/containers
    where the env is already set up.
    """
    if bag_dir.exists():
        shutil.rmtree(bag_dir)
    bag_dir.mkdir(parents=True, exist_ok=True)

    bag_path = bag_dir / "trial_bag"

    # Use the current process environment (already sourced by the user)
    env = os.environ.copy()

    # Kill leftover processes
    kill_ros_processes()

    # --- Start simulation ---
    print(f"    Starting simulation...")
    sim_proc = subprocess.Popen(
        [
            "ros2", "launch", "ee4308_bringup", "proj2_sim.launch.py",
            "headless:=True",
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        # Do NOT use preexec_fn=os.setsid — it can break signal handling
        # in distrobox/containers
    )

    print(f"    Waiting {startup_wait}s for Gazebo startup...")
    time.sleep(startup_wait)

    # Check if sim is still running
    if sim_proc.poll() is not None:
        stderr_out = ""
        if sim_proc.stderr:
            try:
                stderr_out = sim_proc.stderr.read().decode(errors="replace")
            except Exception:
                pass
        print(f"    ERROR: Simulation died during startup (exit code {sim_proc.returncode})")
        if stderr_out:
            for line in stderr_out.strip().split("\n")[-10:]:
                print(f"      {line}")
        kill_ros_processes()
        return False

    # --- Start recording ---
    print(f"    Recording for {sim_duration}s...")
    rec_proc = subprocess.Popen(
        [
            "ros2", "bag", "record",
            "-s", "mcap",
            "-o", str(bag_path),
        ] + RECORD_TOPICS,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # Wait for recording duration, checking sim health periodically
    elapsed = 0.0
    check_interval = 5.0
    while elapsed < sim_duration:
        wait = min(check_interval, sim_duration - elapsed)
        time.sleep(wait)
        elapsed += wait

        if sim_proc.poll() is not None:
            print(f"    WARNING: Simulation died at {elapsed:.0f}s (exit code {sim_proc.returncode})")
            break

    # --- Stop recording ---
    print(f"    Stopping recording...")
    rec_proc.send_signal(signal.SIGINT)
    try:
        rec_proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        rec_proc.kill()
        rec_proc.wait(timeout=5)

    # --- Stop simulation ---
    print(f"    Stopping simulation...")
    sim_proc.send_signal(signal.SIGINT)
    try:
        sim_proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        sim_proc.kill()
        try:
            sim_proc.wait(timeout=5)
        except Exception:
            pass

    # Final cleanup of any orphans
    time.sleep(2)
    kill_ros_processes()

    # Check bag exists
    if not bag_path.exists():
        candidates = list(bag_dir.glob("trial_bag*"))
        if candidates:
            return True
        print(f"    WARNING: No bag found in {bag_dir}")
        return False

    return True


# ===========================================================================
# RMSE computation
# ===========================================================================
def compute_rmse_from_bag(bag_dir: Path) -> dict:
    """Read the bag and compute RMSE between /drone/odom and /drone/true_odom."""
    bag_path = None
    for candidate in bag_dir.iterdir():
        if candidate.is_dir() and (candidate / "metadata.yaml").exists():
            bag_path = candidate
            break
    if bag_path is None:
        if (bag_dir / "metadata.yaml").exists():
            bag_path = bag_dir
        else:
            raise FileNotFoundError(f"No bag found in {bag_dir}")

    typestore = get_typestore(Stores.ROS2_JAZZY)

    est_data = {"times": [], "x": [], "y": [], "z": [],
                "qx": [], "qy": [], "qz": [], "qw": []}
    true_data = {"times": [], "x": [], "y": [], "z": [],
                 "qx": [], "qy": [], "qz": [], "qw": []}

    odom_fields = [
        ("pose.pose.position.x", "x"),
        ("pose.pose.position.y", "y"),
        ("pose.pose.position.z", "z"),
        ("pose.pose.orientation.x", "qx"),
        ("pose.pose.orientation.y", "qy"),
        ("pose.pose.orientation.z", "qz"),
        ("pose.pose.orientation.w", "qw"),
    ]

    with Reader(bag_path) as reader:
        conn_map = {}
        for conn in reader.connections:
            if conn.topic in ["/drone/odom", "/drone/true_odom"]:
                conn_map.setdefault(conn.topic, []).append(conn)

        if "/drone/odom" not in conn_map or "/drone/true_odom" not in conn_map:
            raise ValueError(f"Missing topics. Found: {list(conn_map.keys())}")

        conns = [c for topic_conns in conn_map.values() for c in topic_conns]

        for conn, timestamp_ns, rawdata in reader.messages(connections=conns):
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            t = get_stamp_sec(msg)
            if t is None:
                t = timestamp_ns * 1e-9

            target = est_data if conn.topic == "/drone/odom" else true_data
            target["times"].append(t)
            for field_path, key in odom_fields:
                try:
                    target[key].append(float(resolve_field(msg, field_path)))
                except AttributeError:
                    target[key].append(np.nan)

    for d in [est_data, true_data]:
        for k in d:
            d[k] = np.array(d[k])

    if len(est_data["times"]) < 10 or len(true_data["times"]) < 10:
        raise ValueError(f"Insufficient data: est={len(est_data['times'])}, "
                         f"true={len(true_data['times'])}")

    # Filter warmup
    t0 = min(est_data["times"][0], true_data["times"][0])
    for d in [est_data, true_data]:
        mask = d["times"] >= (t0 + WARMUP_SECONDS)
        for k in d:
            d[k] = d[k][mask]

    if len(est_data["times"]) < 5 or len(true_data["times"]) < 5:
        raise ValueError("Insufficient data after warmup filtering")

    # Interpolate GT to est timestamps
    sort_idx = np.argsort(true_data["times"])
    true_t = true_data["times"][sort_idx]
    est_t = est_data["times"]
    mask = (est_t >= true_t[0]) & (est_t <= true_t[-1])
    est_t_valid = est_t[mask]

    if len(est_t_valid) < 5:
        raise ValueError("Insufficient overlapping timestamps")

    errors = {}
    for key in ["x", "y", "z"]:
        true_interp = np.interp(est_t_valid, true_t, true_data[key][sort_idx])
        est_vals = est_data[key][mask]
        valid = np.isfinite(est_vals) & np.isfinite(true_interp)
        if np.sum(valid) < 5:
            errors[key] = 999.0
            continue
        errors[key] = float(np.sqrt(np.mean((est_vals[valid] - true_interp[valid]) ** 2)))

    # Yaw RMSE
    for qk in ["qx", "qy", "qz", "qw"]:
        true_data[f"{qk}_i"] = np.interp(est_t_valid, true_t, true_data[qk][sort_idx])

    true_yaw = quat_to_yaw(true_data["qx_i"], true_data["qy_i"],
                            true_data["qz_i"], true_data["qw_i"])
    est_yaw = quat_to_yaw(est_data["qx"][mask], est_data["qy"][mask],
                           est_data["qz"][mask], est_data["qw"][mask])
    yaw_err = limit_angle(est_yaw - true_yaw)
    valid_yaw = np.isfinite(yaw_err)
    errors["yaw"] = float(np.sqrt(np.mean(yaw_err[valid_yaw] ** 2))) if np.sum(valid_yaw) > 5 else 999.0

    errors["rmse_3d"] = float(np.sqrt(errors["x"]**2 + errors["y"]**2 + errors["z"]**2))
    errors["rmse_total"] = float(np.sqrt(errors["rmse_3d"]**2 + errors["yaw"]**2))

    return errors


# ===========================================================================
# Optuna objective
# ===========================================================================
def create_objective(workspace, yaml_path, sim_duration, startup_wait,
                     bag_base_dir, objective_metric, n_repeats):
    def objective(trial: optuna.Trial) -> float:
        trial_num = trial.number
        print(f"\n{'='*60}")
        print(f"Trial {trial_num} ({n_repeats} repeat(s))")
        print(f"{'='*60}")

        # Sample parameters — tie x/y together since they're symmetric
        var_imu_xy = trial.suggest_float("var_imu_xy", 1e-4, 1.0, log=True)
        var_gps_xy = trial.suggest_float("var_gps_xy", 1e-6, 1.0, log=True)

        params = {
            "var_imu_x": var_imu_xy,
            "var_imu_y": var_imu_xy,
            "var_imu_z": trial.suggest_float("var_imu_z", 1e-4, 1.0, log=True),
            "var_imu_a": trial.suggest_float("var_imu_a", 1e-6, 0.1, log=True),
            "var_gps_x": var_gps_xy,
            "var_gps_y": var_gps_xy,
            "var_gps_z": trial.suggest_float("var_gps_z", 1e-6, 1.0, log=True),
            "var_magnet": trial.suggest_float("var_magnet", 1e-4, 10.0, log=True),
            "var_sonar": trial.suggest_float("var_sonar", 1e-4, 1.0, log=True),
            "var_baro": trial.suggest_float("var_baro", 1e-3, 10.0, log=True),
        }

        print(f"  Parameters:")
        for k, v in params.items():
            if k in ("var_imu_y", "var_gps_y"):
                continue  # skip duplicates in print
            print(f"    {k}: {v:.6e}")
        print(f"    var_imu_xy: {var_imu_xy:.6e}  (shared x/y)")
        print(f"    var_gps_xy: {var_gps_xy:.6e}  (shared x/y)")

        update_yaml_params(yaml_path, params)

        # Run multiple repeats to average out simulation noise
        all_metrics = []
        for rep in range(n_repeats):
            bag_dir = bag_base_dir / f"trial_{trial_num:04d}_rep{rep}"

            try:
                success = launch_sim_and_record(
                    workspace=workspace,
                    bag_dir=bag_dir,
                    sim_duration=sim_duration,
                    startup_wait=startup_wait,
                )

                if not success:
                    print(f"    Repeat {rep}: FAILED (sim/recording)")
                    continue

                errors = compute_rmse_from_bag(bag_dir)
                all_metrics.append(errors)

                metric_val = errors.get(objective_metric, errors["rmse_total"])
                print(f"    Repeat {rep}: {objective_metric}={metric_val:.4f}  "
                      f"(x={errors['x']:.4f} y={errors['y']:.4f} "
                      f"z={errors['z']:.4f} yaw={errors['yaw']:.4f})")

            except Exception as e:
                print(f"    Repeat {rep}: ERROR: {e}")
                continue

        if not all_metrics:
            print(f"  All repeats failed!")
            return 999.0

        # Average across repeats
        avg_errors = {}
        for key in all_metrics[0].keys():
            values = [m[key] for m in all_metrics if np.isfinite(m[key])]
            if values:
                avg_errors[key] = float(np.mean(values))
                if len(values) > 1:
                    avg_errors[f"{key}_std"] = float(np.std(values))
            else:
                avg_errors[key] = 999.0

        print(f"  Averaged results ({len(all_metrics)} successful repeats):")
        for k in ["x", "y", "z", "yaw", "rmse_3d", "rmse_total"]:
            std_key = f"{k}_std"
            std_str = f" ± {avg_errors[std_key]:.4f}" if std_key in avg_errors else ""
            print(f"    {k}: {avg_errors[k]:.4f}{std_str}")

        for k, v in avg_errors.items():
            trial.set_user_attr(k, v)
        trial.set_user_attr("n_successful_repeats", len(all_metrics))

        metric = avg_errors.get(objective_metric, avg_errors["rmse_total"])
        return metric if np.isfinite(metric) else 999.0

    return objective


# ===========================================================================
# Main
# ===========================================================================
def main():
    parser = argparse.ArgumentParser(description="Automated KF variance tuner")
    parser.add_argument("--workspace", type=str, default=os.path.expanduser("~/ee4308"))
    parser.add_argument("--n-trials", type=int, default=50)
    parser.add_argument("--sim-duration", type=float, default=60.0)
    parser.add_argument("--startup-wait", type=float, default=12.0,
                        help="Seconds to wait for Gazebo to start (default: 12)")
    parser.add_argument("--objective", type=str, default="rmse_3d",
                        choices=["rmse_total", "rmse_3d", "x", "y", "z", "yaw"])
    parser.add_argument("--study-name", type=str, default="kf_variance_tuning")
    parser.add_argument("--bag-dir", type=str, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--n-repeats", type=int, default=2,
                        help="Number of sim repeats per trial to average out noise (default: 2)")
    args = parser.parse_args()

    workspace = Path(args.workspace).expanduser().resolve()
    yaml_path = workspace / PARAMS_REL_PATH

    if not yaml_path.exists():
        sys.exit(f"ERROR: params file not found at {yaml_path}")

    bag_base_dir = Path(args.bag_dir) if args.bag_dir else Path("tmp/kf_tuning_bags")
    bag_base_dir.mkdir(parents=True, exist_ok=True)

    # Back up original yaml
    backup_path = yaml_path.with_suffix(".yaml.backup")
    if not backup_path.exists():
        shutil.copy2(yaml_path, backup_path)
        print(f"Backed up original params to {backup_path}")

    print(f"Workspace:      {workspace}")
    print(f"Params file:    {yaml_path}")
    print(f"Bag directory:  {bag_base_dir}")
    print(f"Trials:         {args.n_trials}")
    print(f"Sim duration:   {args.sim_duration}s")
    print(f"Startup wait:   {args.startup_wait}s")
    print(f"Objective:      {args.objective}")
    print(f"Repeats/trial:  {args.n_repeats}")
    print()

    # Kill leftover processes before starting
    print("Cleaning up leftover processes...")
    kill_ros_processes()

    storage = f"sqlite:///{bag_base_dir / 'optuna_study.db'}"

    if args.resume:
        study = optuna.load_study(study_name=args.study_name, storage=storage)
        print(f"Resumed study with {len(study.trials)} existing trials")
    else:
        study = optuna.create_study(
            study_name=args.study_name,
            storage=storage,
            direction="minimize",
            load_if_exists=True,
        )

    objective = create_objective(
        workspace=workspace,
        yaml_path=yaml_path,
        sim_duration=args.sim_duration,
        startup_wait=args.startup_wait,
        bag_base_dir=bag_base_dir,
        objective_metric=args.objective,
        n_repeats=args.n_repeats,
    )

    try:
        study.optimize(objective, n_trials=args.n_trials)
    except KeyboardInterrupt:
        print("\n\nInterrupted.")

    # Results
    completed = [t for t in study.trials
                 if t.state == optuna.trial.TrialState.COMPLETE and t.value < 900]

    print(f"\n{'='*70}")
    print(f"RESULTS ({len(completed)} successful / {len(study.trials)} total)")
    print(f"{'='*70}")

    if not completed:
        print("No successful trials!")
        return

    best = study.best_trial
    print(f"\nBest trial #{best.number}: {args.objective} = {best.value:.6f}")

    # Expand tied params for proj2.yaml
    best_yaml_params = {}
    for k, v in best.params.items():
        if k == "var_imu_xy":
            best_yaml_params["var_imu_x"] = v
            best_yaml_params["var_imu_y"] = v
        elif k == "var_gps_xy":
            best_yaml_params["var_gps_x"] = v
            best_yaml_params["var_gps_y"] = v
        else:
            best_yaml_params[k] = v

    print(f"\nBest parameters (paste into proj2.yaml):")
    print(f"    estimator:")
    print(f"      ros__parameters:")
    for k, v in sorted(best_yaml_params.items()):
        print(f"        {k}: {v:.6e}")

    if best.user_attrs:
        print(f"\n  All metrics:")
        for k, v in sorted(best.user_attrs.items()):
            print(f"    {k}: {v:.4f}")

    # Write best params
    update_yaml_params(yaml_path, best_yaml_params)
    print(f"\nBest params written to {yaml_path}")
    print(f"Original backup at {backup_path}")

    # Save results
    results_file = bag_base_dir / "best_params.yaml"
    with open(results_file, "w") as f:
        yaml.dump({
            "trial": best.number,
            "objective": args.objective,
            "value": float(best.value),
            "params_optuna": {k: float(v) for k, v in best.params.items()},
            "params_yaml": {k: float(v) for k, v in best_yaml_params.items()},
            "metrics": {k: float(v) for k, v in best.user_attrs.items()},
        }, f, default_flow_style=False)

    # Top 5
    sorted_trials = sorted(completed, key=lambda t: t.value)[:5]
    print(f"\nTop 5 trials:")
    for t in sorted_trials:
        print(f"  #{t.number}: {args.objective}={t.value:.4f}")

    # Optuna plots
    try:
        from optuna.visualization import plot_optimization_history, plot_param_importances
        plots_dir = bag_base_dir / "plots"
        plots_dir.mkdir(exist_ok=True)
        plot_optimization_history(study).write_html(str(plots_dir / "history.html"))
        plot_param_importances(study).write_html(str(plots_dir / "importances.html"))
        print(f"\nPlots saved to {plots_dir}/")
    except Exception:
        pass


if __name__ == "__main__":
    main()