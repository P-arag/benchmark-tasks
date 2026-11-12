"""Scripted full-score demo, OPEN LOOP (targets from geometry.py only; object_jitter=0).

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--one-button]

!!! Run reach_check.py first. If the buttons are out of reach this demo cannot work as written.

1. Each arm comes in from its own side, fingers closed and pointing inward, and presses its button in (both at once).
2. Still pressing, both arms lift the hinge 10 cm: the frame comes off the table and the front legs swing back
   under their own weight past 8 degrees (the fold stays free once past that).
3. The right arm lets its button go and pushes the front legs' footrest back toward the rear legs until the
   storage latch catches; the left arm keeps holding the hinge.
4. The left arm lowers the folded stroller onto its wheels and lets go.

``--one-button`` presses only the left button: the fold must stay locked (expect 0).

# CHECK (run): everything here, starting with reach. If the hinge can be reached only from above, swap step 1 for a
# top-down pinch that squeezes both buttons with one gripper per side (still two hands).
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import PAD_TIP, run, together, turn, wait  # noqa: E402

TOP = g.TOP
HINGE = np.array([g.ORIGIN_X, g.ORIGIN_Y, TOP + g.HINGE_Z])
BTN_OUT = g.HALF_W + 0.015 + g.BUTTON[0]                 # the buttons' outer faces, from the middle


def side_rot(sign: int) -> np.ndarray:
    """Fingers horizontal, pointing at the stroller's middle from the +y (sign=+1) or -y side."""
    from yam_arena.demo import grip_rotation

    return turn([1, 0, 0], sign * math.pi / 2) @ grip_rotation(math.pi / 2)


def press(arm, sign: int, depth: float = 0.007):
    face = HINGE + [0, sign * BTN_OUT, 0]
    yield from arm.move(face + [0, sign * (0.06 + PAD_TIP), 0], side_rot(sign), 2.5, grip=0.0)
    yield from arm.move(face + [0, sign * (PAD_TIP - depth), 0], None, 1.2)


def lift(arm, dz: float, seconds: float):
    yield from arm.move(arm.pos + [0, 0, dz], None, seconds)


def fold_front(ctx):
    from yam_arena.demo import grip_rotation

    arm = ctx.right
    yield from arm.move(arm.pos + [0, -0.06, 0.0], None, 0.8)                 # off the button
    foot = HINGE + [g.FRONT_WHEEL_X * 0.75 - 0.03, 0, -g.HINGE_Z * 0.7 + 0.10]
    rot = turn([0, 1, 0], -math.pi / 2) @ grip_rotation(math.pi / 2)        # fingers pointing -x, closed
    yield from arm.move(foot + [0.08, 0, 0.0], rot, 2.0, grip=0.0)
    yield from arm.move(foot + [0.01, 0, 0.0], None, 1.0)
    yield from arm.move(foot + [-0.25, 0, 0.05], None, 3.0)                  # swing the front legs back


def script(ctx, both: bool = True):
    L, R = ctx.left, ctx.right
    gens = [press(L, +1)] + ([press(R, -1)] if both else [])
    yield from together(*gens)
    yield from wait(0.5)
    gens = [lift(L, 0.10, 2.0)] + ([lift(R, 0.10, 2.0)] if both else [])
    yield from together(*gens)
    if both:
        yield from fold_front(ctx)
    yield from lift(L, -0.10, 2.0)
    yield from L.move(L.pos + [0, 0.08, 0], None, 1.0)
    if both:
        yield from R.move(R.pos + [-0.08, 0, 0.05], None, 1.0)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE                                                        # logging only
    l, r = S["pressed"]
    return (f"buttons {l * 1000:3.1f}/{r * 1000:3.1f} mm unlocked {S['unlocked']} fold "
            f"{math.degrees(S['fold']):5.1f} latched {S['latched']} tilt {math.degrees(S['tilt']):4.1f} "
            f"outcome {ctx.task.outcome()}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--one-button", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, not a.one_button), max_steps=a.max_steps, record=not a.no_record,
        log=log, log_every=20)
