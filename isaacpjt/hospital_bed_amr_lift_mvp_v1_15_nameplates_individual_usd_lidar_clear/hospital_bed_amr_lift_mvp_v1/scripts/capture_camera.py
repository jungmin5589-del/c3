#!/usr/bin/env python3
"""Capture RGB and depth frames from the front and rear AMR cameras."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

try:
    from isaacsim.simulation_app import SimulationApp
except ImportError:
    from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--stage", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--camera",
        choices=("front", "rear", "both"),
        default="both",
        help="Camera to capture. Default: both",
    )
    parser.add_argument(
        "--amr",
        default="AMR1",
        help="Fleet unit name or namespace. Default: AMR1",
    )
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


ARGS = parse_args()
APP = SimulationApp({"headless": ARGS.headless, "renderer": "RayTracedLighting"})

import carb
import numpy as np
import omni.timeline
import omni.usd
from PIL import Image
from isaacsim.sensors.camera import Camera


def as_numpy(value):
    if value is None:
        return None
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def save_camera_frame(camera: Camera, output_dir: Path, prefix: str) -> None:
    rgba = as_numpy(camera.get_rgba())
    depth = as_numpy(camera.get_depth())

    if rgba is None or rgba.size == 0:
        raise RuntimeError(f"{prefix}: camera returned no RGBA data")
    rgb = np.clip(rgba[..., :3], 0, 255).astype(np.uint8)
    rgb_path = output_dir / f"{prefix}_rgb.png"
    Image.fromarray(rgb).save(rgb_path)

    if depth is None or depth.size == 0:
        raise RuntimeError(f"{prefix}: camera returned no depth data")
    depth = depth.astype(np.float32)
    depth_npy = output_dir / f"{prefix}_depth.npy"
    np.save(depth_npy, depth)

    finite = np.isfinite(depth)
    depth_png = output_dir / f"{prefix}_depth_preview.png"
    preview = np.zeros(depth.shape, dtype=np.uint8)
    if finite.any():
        valid = depth[finite]
        near = float(np.percentile(valid, 2.0))
        far = float(np.percentile(valid, 98.0))
        if far <= near:
            far = near + 1.0
        normalized = 1.0 - np.clip((depth - near) / (far - near), 0.0, 1.0)
        preview[finite] = (normalized[finite] * 255.0).astype(np.uint8)
    Image.fromarray(preview).save(depth_png)

    print(f"[camera] {prefix} RGB: {rgb_path}")
    print(f"[camera] {prefix} depth NPY: {depth_npy}")
    print(f"[camera] {prefix} depth preview: {depth_png}")
    print(f"[camera] {prefix} RGB shape={rgb.shape}, depth shape={depth.shape}")


def main() -> int:
    cfg = json.loads(Path(ARGS.config).read_text(encoding="utf-8"))
    stage_path = Path(ARGS.stage).expanduser().resolve()
    output_dir = Path(ARGS.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not stage_path.exists():
        raise FileNotFoundError(stage_path)

    unit = next(
        (
            item
            for item in cfg["fleet"]
            if str(item.get("name", "")).lower() == ARGS.amr.lower()
            or str(item.get("namespace", "")).lower() == ARGS.amr.lower()
        ),
        None,
    )
    if unit is None:
        raise ValueError(f"AMR not found in config: {ARGS.amr}")

    context = omni.usd.get_context()
    if not context.open_stage(str(stage_path)):
        raise RuntimeError(f"Could not open stage: {stage_path}")
    for _ in range(90):
        APP.update()

    sensor_cfg = cfg["sensors"]
    resolution = tuple(map(int, sensor_cfg["camera_resolution"]))
    requested = {"front", "rear"} if ARGS.camera == "both" else {ARGS.camera}
    cameras = []
    for definition in sensor_cfg.get("cameras", []):
        name = str(definition["name"])
        if name not in requested:
            continue
        prim_path = f"{unit['root_path']}/{definition['prim_suffix']}"
        camera = Camera(
            prim_path=prim_path,
            name=f"capture_{unit['namespace']}_{name}",
            frequency=float(sensor_cfg["camera_frequency_hz"]),
            resolution=resolution,
        )
        camera.initialize()
        camera.add_distance_to_image_plane_to_frame()
        cameras.append((name, camera))

    if not cameras:
        raise RuntimeError("No requested camera definitions were found")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(180):
        APP.update()

    for name, camera in cameras:
        save_camera_frame(camera, output_dir, f"{unit['namespace']}_{name}_camera")

    timeline.stop()
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception as exc:
        carb.log_error(str(exc))
        print(f"[error] {exc}", file=sys.stderr)
    finally:
        APP.close()
    raise SystemExit(code)
