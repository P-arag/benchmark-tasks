"""Write the stroller USD asset from geometry.py.

    python build_assets.py          # Isaac Sim's Python (pxr); no simulator needed

assets/stroller.usda: a FLOATING articulation (it stands free on its wheels). Root ``rear`` (rear legs and wheels, push
handles, seat back, hinge blocks); ``front`` on revolute ``fold`` (about y at the main hinge; positive folds the front
legs back onto the rear legs); ``button_l`` / ``button_r`` on prismatic ``press_l`` / ``press_r`` (along y, inward)
on the hinge blocks. The seat fabric and canopy are visual only.

# CHECK (run): Play with the stroller open: it must stand without rolling or sagging (the fold is locked by task.py's
# limits at 0). Wheels are fixed cylinders with low friction (no wheel joints): if it skids instead of rolling when
# folding, add revolute wheel joints.
"""

from __future__ import annotations

import math
from pathlib import Path

import geometry as g
from usdkit import articulation_root, body, box, capsule, cylinder, joint, material, no_collide, quat_axis_angle, stage_for

OUT = Path(__file__).resolve().parent / "assets"


def tube(stage, path, p0, p1, mat):
    """A tube between two points in the xz plane (same y), as a capsule."""
    dx, dz = p1[0] - p0[0], p1[2] - p0[2]
    length = math.hypot(dx, dz)
    mid = ((p0[0] + p1[0]) / 2, p0[1], (p0[2] + p1[2]) / 2)
    ang = math.atan2(dz, dx)
    return capsule(stage, path, g.TUBE_R, length, mid, axis="X", mat=mat, rot_wxyz=quat_axis_angle((0, 1, 0), -ang))


def build() -> None:
    stage = stage_for(OUT, "stroller")
    r = "/stroller"
    alu = material(stage, f"{r}/Looks/alu", (0.75, 0.76, 0.78), roughness=0.3, metallic=0.9)
    black = material(stage, f"{r}/Looks/black", (0.08, 0.08, 0.09), roughness=0.6)
    fabric = material(stage, f"{r}/Looks/fabric", (0.20, 0.30, 0.45), roughness=0.9)
    hz = g.HINGE_Z
    # origin: on the table under the hinge
    rear = f"{r}/rear"
    body(stage, rear, g.FRAME_MASS_REAR)
    articulation_root(stage, rear, fixed=False)
    for s, y in (("l", g.HALF_W), ("r", -g.HALF_W)):
        wheel = (g.REAR_WHEEL_X, y, g.WHEEL_R)
        tube(stage, f"{rear}/leg_{s}", wheel, (0, y, hz), alu)
        tube(stage, f"{rear}/handle_{s}", (0, y, hz), (g.HANDLE_TOP[0], y, g.HANDLE_TOP[1]), alu)
        cylinder(stage, f"{rear}/wheel_{s}", g.WHEEL_R, g.WHEEL_W, wheel, axis="Y", mat=black)
        capsule(stage, f"{rear}/grip_{s}", 0.015, 0.08, (g.HANDLE_TOP[0], y, g.HANDLE_TOP[1]), axis="Z", mat=black)
        box(stage, f"{rear}/hinge_block_{s}", (0.05, 0.03, 0.05), (0, y, hz), mat=black)
    cylinder(stage, f"{rear}/axle", 0.006, 2 * g.HALF_W, (g.REAR_WHEEL_X, 0, g.WHEEL_R), axis="Y", mat=alu)
    box(stage, f"{rear}/seat_back", (0.02, 2 * g.HALF_W - 0.04, 0.35), (-0.10, 0, hz + 0.08), collide=False,
        mat=fabric, rot_wxyz=quat_axis_angle((0, 1, 0), math.radians(-20)))

    front = f"{r}/front"
    body(stage, front, g.FRAME_MASS_FRONT, (0, 0, hz))
    for s, y in (("l", g.HALF_W - 0.035), ("r", -g.HALF_W + 0.035)):
        wheel = (g.FRONT_WHEEL_X, y, g.WHEEL_R - hz)
        tube(stage, f"{front}/leg_{s}", (0, y, 0), wheel, alu)
        cylinder(stage, f"{front}/wheel_{s}", g.WHEEL_R * 0.8, g.WHEEL_W, (wheel[0], y, wheel[2] - 0.2 * g.WHEEL_R),
                 axis="Y", mat=black)
    box(stage, f"{front}/footrest", (0.06, 2 * g.HALF_W - 0.10, 0.01), (g.FRONT_WHEEL_X * 0.75, 0, -hz * 0.7),
        mat=black)
    box(stage, f"{front}/seat", (0.30, 2 * g.HALF_W - 0.06, 0.01), (0.12, 0, -0.05), collide=False, mat=fabric)
    joint(stage, f"{r}/joints/fold", "angular", rear, front, (0, 0, hz), axis="Y", limits=(0.0, g.FOLD_MAX))
    no_collide(stage, front, rear)

    for s, sign in (("l", 1), ("r", -1)):
        b = f"{r}/button_{s}"
        y = sign * (g.HALF_W + 0.015 + g.BUTTON[0] / 2)
        body(stage, b, 0.01, (0, y, hz))
        box(stage, f"{b}/cap", (g.BUTTON[1], g.BUTTON[0], g.BUTTON[2]), (0, 0, 0), color=(0.85, 0.15, 0.10))
        # axis y: pressing moves the button toward the middle (-y on the left, +y on the right)
        lim = (-g.BUTTON_TRAVEL, 0.0) if sign > 0 else (0.0, g.BUTTON_TRAVEL)
        joint(stage, f"{r}/joints/press_{s}", "linear", rear, b, (0, y, hz), axis="Y", limits=lim)
        no_collide(stage, b, rear)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    build()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
