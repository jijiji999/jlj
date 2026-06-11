#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path


DEFAULT_SOURCE = Path("/home/sliouzhou04/jlj/pkl/zoulu.csv")
DEFAULT_OUTPUT = (
    Path(__file__).resolve().parents[1] / "config" / "whole_body_zoulu.csv"
)

WHOLE_BODY_JOINTS = [
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Convert a raw whole-body trajectory CSV with root pose plus 29 joint "
            "angles into the dm_test time+joints CSV format."
        )
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--root-columns", type=int, default=7)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.fps <= 0.0:
        raise ValueError("--fps must be > 0")
    if args.root_columns < 0:
        raise ValueError("--root-columns must be >= 0")
    if not args.source.exists():
        raise FileNotFoundError(f"source CSV not found: {args.source}")

    expected_columns = args.root_columns + len(WHOLE_BODY_JOINTS)
    rows = []
    with args.source.open("r", newline="") as source_handle:
        reader = csv.reader(source_handle)
        for row_index, row in enumerate(reader):
            if not row:
                continue
            if len(row) != expected_columns:
                raise ValueError(
                    f"source row {row_index + 1} has {len(row)} columns, "
                    f"expected {expected_columns}"
                )

            joint_values = row[args.root_columns:]
            time_from_start = row_index / args.fps
            rows.append([f"{time_from_start:.9f}", *joint_values])

    if not rows:
        raise ValueError(f"source CSV has no data rows: {args.source}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as output_handle:
        writer = csv.writer(output_handle)
        writer.writerow(["time", *WHOLE_BODY_JOINTS])
        writer.writerows(rows)

    duration = (len(rows) - 1) / args.fps
    print(
        f"wrote {args.output} with {len(rows)} rows at {args.fps:g} FPS "
        f"(duration {duration:.3f}s)"
    )


if __name__ == "__main__":
    main()
