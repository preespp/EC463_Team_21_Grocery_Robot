#!/usr/bin/env python3
"""
Generate a quick markdown comparison report for two Nav2 parameter profiles.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, List, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise SystemExit("PyYAML is required (python3-yaml).") from exc


KEY_PATHS = [
    "controller_server.ros__parameters.controller_frequency",
    "controller_server.ros__parameters.progress_checker.required_movement_radius",
    "controller_server.ros__parameters.progress_checker.movement_time_allowance",
    "controller_server.ros__parameters.FollowPath.max_vel_x",
    "controller_server.ros__parameters.FollowPath.max_vel_y",
    "controller_server.ros__parameters.FollowPath.max_vel_theta",
    "controller_server.ros__parameters.FollowPath.max_speed_xy",
    "controller_server.ros__parameters.FollowPath.acc_lim_x",
    "controller_server.ros__parameters.FollowPath.acc_lim_y",
    "controller_server.ros__parameters.FollowPath.acc_lim_theta",
    "controller_server.ros__parameters.FollowPath.vx_samples",
    "controller_server.ros__parameters.FollowPath.vy_samples",
    "controller_server.ros__parameters.FollowPath.vtheta_samples",
    "controller_server.ros__parameters.FollowPath.BaseObstacle.scale",
    "local_costmap.local_costmap.ros__parameters.update_frequency",
    "local_costmap.local_costmap.ros__parameters.width",
    "local_costmap.local_costmap.ros__parameters.height",
    "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius",
    "local_costmap.local_costmap.ros__parameters.voxel_layer.cloud.obstacle_max_range",
]


def load_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def get_path(data: dict, path: str) -> Any:
    current: Any = data
    for part in path.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def make_rows(base: dict, target: dict) -> List[Tuple[str, Any, Any]]:
    rows: List[Tuple[str, Any, Any]] = []
    for path in KEY_PATHS:
        rows.append((path, get_path(base, path), get_path(target, path)))
    return rows


def render_markdown(
    base_name: str,
    target_name: str,
    rows: List[Tuple[str, Any, Any]],
) -> str:
    lines = [
        f"# Nav2 Profile Report: {target_name} vs {base_name}",
        "",
        "| Parameter | Base | Target |",
        "|---|---:|---:|",
    ]
    for path, base_value, target_value in rows:
        lines.append(f"| `{path}` | `{base_value}` | `{target_value}` |")

    lines.extend(
        [
            "",
            "## Runtime Regression Checklist",
            "- CPU usage (avg/peak): _fill after run_",
            "- Memory usage (avg/peak): _fill after run_",
            "- Controller frequency dropouts: _fill after run_",
            "- Dynamic obstacle success rate: _fill after run_",
            "- Manual override latency test result: _fill after run_",
        ]
    )
    return "\n".join(lines) + "\n"


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare two Nav2 profile YAML files.")
    parser.add_argument("--base", required=True, help="Base profile yaml path")
    parser.add_argument("--target", required=True, help="Target profile yaml path")
    parser.add_argument("--output", default="", help="Optional markdown output path")
    args = parser.parse_args(argv)

    base_path = Path(args.base)
    target_path = Path(args.target)

    base = load_yaml(base_path)
    target = load_yaml(target_path)
    rows = make_rows(base, target)
    report = render_markdown(base_path.name, target_path.name, rows)

    if args.output:
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
