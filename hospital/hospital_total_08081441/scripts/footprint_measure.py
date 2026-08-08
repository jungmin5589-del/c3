#!/usr/bin/env python3
"""Measure AMR, attached bed, and combined Nav2 footprint in base_link coordinates."""

from __future__ import annotations

from typing import Any

from pxr import Gf, Usd, UsdGeom


_PURPOSES = [
    UsdGeom.Tokens.default_,
    UsdGeom.Tokens.render,
    UsdGeom.Tokens.proxy,
]


def _world_matrix(prim: Usd.Prim) -> Gf.Matrix4d:
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )


def _bbox_world_corners(cache: UsdGeom.BBoxCache, prim: Usd.Prim) -> list[Gf.Vec3d]:
    """Return the eight corners of a prim subtree's oriented world bound."""
    if not prim or not prim.IsValid():
        return []

    bbox = cache.ComputeWorldBound(prim)
    rng = bbox.GetRange()
    if rng.IsEmpty():
        return []

    matrix = bbox.GetMatrix()
    minimum = rng.GetMin()
    maximum = rng.GetMax()
    corners: list[Gf.Vec3d] = []
    for x in (minimum[0], maximum[0]):
        for y in (minimum[1], maximum[1]):
            for z in (minimum[2], maximum[2]):
                corners.append(matrix.Transform(Gf.Vec3d(x, y, z)))
    return corners


def _bounds_in_frame(
    world_corners: list[Gf.Vec3d],
    frame_world: Gf.Matrix4d,
    meters_per_unit: float,
) -> dict[str, float]:
    if not world_corners:
        raise RuntimeError("No geometry was found while computing the bound.")

    inverse = frame_world.GetInverse()
    local = [inverse.Transform(point) for point in world_corners]
    xs = [float(point[0]) * meters_per_unit for point in local]
    ys = [float(point[1]) * meters_per_unit for point in local]
    zs = [float(point[2]) * meters_per_unit for point in local]
    return {
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y": min(ys),
        "max_y": max(ys),
        "min_z": min(zs),
        "max_z": max(zs),
    }


def _polygon(bounds: dict[str, float], padding_m: float = 0.0) -> list[list[float]]:
    rear = bounds["min_x"] - padding_m
    front = bounds["max_x"] + padding_m
    right = bounds["min_y"] - padding_m
    left = bounds["max_y"] + padding_m
    return [
        [rear, right],
        [front, right],
        [front, left],
        [rear, left],
    ]


def _format_polygon(points: list[list[float]]) -> str:
    return "[" + ", ".join(
        f"[{x:.3f}, {y:.3f}]" for x, y in points
    ) + "]"


def _print_bound(label: str, bounds: dict[str, float]) -> None:
    length = bounds["max_x"] - bounds["min_x"]
    width = bounds["max_y"] - bounds["min_y"]
    height = bounds["max_z"] - bounds["min_z"]
    print(
        f"[FOOTPRINT] {label}: "
        f"length(X)={length:.3f}m width(Y)={width:.3f}m height(Z)={height:.3f}m"
    )
    print(
        f"[FOOTPRINT] {label} offsets from frame origin: "
        f"rear={-bounds['min_x']:.3f}m front={bounds['max_x']:.3f}m "
        f"right={-bounds['min_y']:.3f}m left={bounds['max_y']:.3f}m"
    )


def report_combined_footprint(
    stage: Usd.Stage,
    controller: Any,
    padding_m: float = 0.08,
) -> dict[str, float] | None:
    """Print the exact current combined footprint relative to AMR base_link."""
    base_prim = controller.base_prim
    amr_root = stage.GetPrimAtPath(controller.root)
    bed_path = controller.magnet.attached_bed_path

    if not bed_path:
        print(
            "[FOOTPRINT] No bed is attached. Move under a bed, press C, "
            "then press M to measure the combined footprint."
        )
        return None

    bed_root = stage.GetPrimAtPath(bed_path)
    if not bed_root or not bed_root.IsValid():
        print(f"[FOOTPRINT] Attached bed prim is invalid: {bed_path}")
        return None

    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        _PURPOSES,
        useExtentsHint=True,
        ignoreVisibility=False,
    )

    base_world = _world_matrix(base_prim)
    bed_world = _world_matrix(bed_root)

    amr_corners = _bbox_world_corners(cache, amr_root)
    bed_corners = _bbox_world_corners(cache, bed_root)

    amr_in_base = _bounds_in_frame(amr_corners, base_world, meters_per_unit)
    bed_in_bed = _bounds_in_frame(bed_corners, bed_world, meters_per_unit)
    bed_in_base = _bounds_in_frame(bed_corners, base_world, meters_per_unit)
    combined = _bounds_in_frame(
        amr_corners + bed_corners,
        base_world,
        meters_per_unit,
    )

    print("\n================ EXACT NAV2 FOOTPRINT MEASUREMENT ================")
    print(f"[FOOTPRINT] stage metersPerUnit={meters_per_unit:g}")
    print(f"[FOOTPRINT] AMR frame=/World/AMR1/base_link (+X forward, +Y left)")
    print(f"[FOOTPRINT] attached bed={bed_path}")
    _print_bound("AMR only in base_link", amr_in_base)
    _print_bound("BED physical/visual bound in bed frame", bed_in_bed)
    _print_bound("BED current position in base_link", bed_in_base)
    _print_bound("COMBINED AMR + BED in base_link", combined)

    raw_polygon = _polygon(combined, 0.0)
    padded_polygon = _polygon(combined, padding_m)
    print("[FOOTPRINT] Nav2 exact polygon without extra padding:")
    print(f'footprint: "{_format_polygon(raw_polygon)}"')
    print("[FOOTPRINT] Recommended conservative polygon with built-in "
          f"{padding_m:.2f}m margin:")
    print(f'footprint: "{_format_polygon(padded_polygon)}"')
    print("footprint_padding: 0.0")
    print(
        "[FOOTPRINT] Copy the SAME footprint into both local_costmap and "
        "global_costmap, and remove robot_radius from both."
    )
    print("=================================================================\n")
    return combined
