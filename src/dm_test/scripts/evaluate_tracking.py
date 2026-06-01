#!/usr/bin/env python3

import argparse
import csv
import math
import sys
from pathlib import Path


DEFAULT_LEFT_ARM_JOINTS = [
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Evaluate dm_test trajectory tracking metrics from CSV logs."
    )
    parser.add_argument(
        "--error-csv",
        help=(
            "CSV exported from /dm_test/error_joint_states. "
            "Expected columns: time,joint,position_error[,velocity_error]"
        ),
    )
    parser.add_argument(
        "--reference-csv",
        help=(
            "CSV for /dm_test/reference_joint_states with columns "
            "time,joint,position[,velocity]"
        ),
    )
    parser.add_argument(
        "--actual-csv",
        help=(
            "CSV for /dm_test/actual_joint_states with columns "
            "time,joint,position[,velocity]"
        ),
    )
    parser.add_argument(
        "--joints",
        nargs="*",
        default=DEFAULT_LEFT_ARM_JOINTS,
        help="Joints to evaluate. Default is the left arm.",
    )
    return parser.parse_args()


def ensure_exists(path_str):
    path = Path(path_str)
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")
    return path


def parse_float(value):
    text = (value or "").strip()
    if not text:
        return None
    lowered = text.lower()
    if lowered in {"nan", "none"}:
        return None
    return float(text)


def load_error_rows(path, target_joints):
    stats = {joint: [] for joint in target_joints}
    velocity_stats = {joint: [] for joint in target_joints}

    with path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"joint", "position_error"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                f"{path} is missing required columns: {', '.join(sorted(missing))}"
            )

        for row in reader:
            joint = (row.get("joint") or "").strip()
            if joint not in stats:
                continue

            position_error = parse_float(row.get("position_error"))
            if position_error is not None:
                stats[joint].append(position_error)

            velocity_error = parse_float(row.get("velocity_error"))
            if velocity_error is not None:
                velocity_stats[joint].append(velocity_error)

    return stats, velocity_stats


def load_joint_series(path, target_joints, value_key):
    series = {joint: [] for joint in target_joints}
    with path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"joint", "time", value_key}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                f"{path} is missing required columns: {', '.join(sorted(missing))}"
            )

        for row in reader:
            joint = (row.get("joint") or "").strip()
            if joint not in series:
                continue

            time_value = parse_float(row.get("time"))
            data_value = parse_float(row.get(value_key))
            if time_value is None or data_value is None:
                continue
            series[joint].append((time_value, data_value))

    for joint in target_joints:
        series[joint].sort(key=lambda item: item[0])
    return series


def nearest_value(sample_list, target_time):
    if not sample_list:
        return None

    best_value = sample_list[0][1]
    best_dt = abs(sample_list[0][0] - target_time)
    for time_value, data_value in sample_list[1:]:
        dt = abs(time_value - target_time)
        if dt < best_dt:
            best_dt = dt
            best_value = data_value
    return best_value


def build_error_from_reference_and_actual(reference_path, actual_path, target_joints):
    reference_series = load_joint_series(reference_path, target_joints, "position")
    actual_series = load_joint_series(actual_path, target_joints, "position")

    position_errors = {joint: [] for joint in target_joints}
    for joint in target_joints:
        for sample_time, actual_position in actual_series[joint]:
            reference_position = nearest_value(reference_series[joint], sample_time)
            if reference_position is None:
                continue
            position_errors[joint].append(actual_position - reference_position)

    return position_errors


def compute_metrics(values):
    if not values:
        return None

    absolute_values = [abs(value) for value in values]
    mae = sum(absolute_values) / len(absolute_values)
    rmse = math.sqrt(sum(value * value for value in values) / len(values))
    max_abs = max(absolute_values)
    return {
        "count": len(values),
        "mae": mae,
        "rmse": rmse,
        "max_abs": max_abs,
    }


def print_table(title, metric_map):
    print(title)
    print(
        f"{'Joint':<24} {'Count':>8} {'MAE(rad)':>12} {'RMSE(rad)':>12} {'MaxAbs(rad)':>14}"
    )
    for joint, metrics in metric_map.items():
        if metrics is None:
            print(f"{joint:<24} {'0':>8} {'-':>12} {'-':>12} {'-':>14}")
            continue
        print(
            f"{joint:<24} {metrics['count']:>8d} "
            f"{metrics['mae']:>12.5f} {metrics['rmse']:>12.5f} {metrics['max_abs']:>14.5f}"
        )
    print()


def summarize_overall(metric_map):
    valid_metrics = [item for item in metric_map.values() if item is not None]
    if not valid_metrics:
        return None

    return {
        "joint_count": len(valid_metrics),
        "avg_mae": sum(item["mae"] for item in valid_metrics) / len(valid_metrics),
        "avg_rmse": sum(item["rmse"] for item in valid_metrics) / len(valid_metrics),
        "worst_max_abs": max(item["max_abs"] for item in valid_metrics),
    }


def main():
    args = parse_args()

    if not args.error_csv and not (args.reference_csv and args.actual_csv):
        print(
            "Need either --error-csv or both --reference-csv and --actual-csv.",
            file=sys.stderr,
        )
        return 2

    target_joints = args.joints

    if args.error_csv:
        error_path = ensure_exists(args.error_csv)
        position_errors, velocity_errors = load_error_rows(error_path, target_joints)
    else:
        reference_path = ensure_exists(args.reference_csv)
        actual_path = ensure_exists(args.actual_csv)
        position_errors = build_error_from_reference_and_actual(
            reference_path, actual_path, target_joints
        )
        velocity_errors = {joint: [] for joint in target_joints}

    position_metrics = {
        joint: compute_metrics(position_errors[joint]) for joint in target_joints
    }
    velocity_metrics = {
        joint: compute_metrics(velocity_errors[joint]) for joint in target_joints
    }

    print_table("Position Tracking Metrics", position_metrics)
    if any(metrics is not None for metrics in velocity_metrics.values()):
        print_table("Velocity Tracking Metrics", velocity_metrics)

    overall = summarize_overall(position_metrics)
    if overall is None:
        print("No valid samples were found for the selected joints.", file=sys.stderr)
        return 1

    print("Overall Summary")
    print(f"  joints_evaluated : {overall['joint_count']}")
    print(f"  average_mae_rad  : {overall['avg_mae']:.5f}")
    print(f"  average_rmse_rad : {overall['avg_rmse']:.5f}")
    print(f"  worst_max_abs_rad: {overall['worst_max_abs']:.5f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
