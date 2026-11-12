"""Scripted full-score demo, OPEN LOOP (targets from geometry.py only; object_jitter=0).

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--no-hold] [--yank]

1. Left arm grips the tin's body from its left side, 5 cm up, and holds it for the whole run.
2. Right arm, fingers closed and pointing down, slides one fingertip under the ring end from the front and lifts it
   round the rivet to 70 degrees (the score cracks at 55).
3. It then pulls the ring up and back over the tin, carrying the lid round its far edge to 120 degrees, and lets go.

``--no-hold`` skips step 1 (the tin is dragged and tips: expect a spill, 0). ``--yank`` peels at 4x the speed with the
tin held (should still be fine, or show that the tear damping needs raising).

# CHECK (run):
#   a) Getting under the ring: the fingertip must fit in the ~5.5 mm gap under the ring end (RECESS_DEPTH +
#      RING_RAISE). If it can't, see the note at the end of build_assets.py, or have the demo push the ring end
#      up from below through the finger hole (needs a narrower fingertip pose).
#   b) HOLD_ROT: fingers horizontal, closing across the tin along x. Flip the sign of the 90-degree turn if they
#      come out pointing the wrong way.
#   c) The tin is free-standing: if the lift drags it toward the arms, grip lower (HOLD_Z) or squeeze harder.
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
BASE = np.array([g.CAN_X, g.CAN_Y, TOP])
HOLD_Z = 0.05
HINGE = BASE + [g.HINGE_X, 0, g.LID_Z]
RIVET_L = np.array([g.RIVET_X - g.HINGE_X, 0, g.LID_T])        # rivet in the lid frame (origin at the hinge)
RING_LEN = g.TAB_L - g.TAB_NOSE


def ring_point(tab: float, peel: float) -> np.ndarray:
    """World position of the ring end for given tab and lid angles (both about +y)."""
    q = RIVET_L + RING_LEN * np.array([-math.cos(tab), 0, math.sin(tab)])
    c, s = math.cos(peel), math.sin(peel)
    return HINGE + np.array([q[0] * c + q[2] * s, 0.0, -q[0] * s + q[2] * c])


def hold_rot() -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return turn([1, 0, 0], -math.pi / 2) @ grip_rotation(math.pi / 2)


def hold_can(ctx):
    arm = ctx.left
    at = BASE + [0, 0, HOLD_Z]
    side = at + [0, g.CAN_R + 0.06, 0]
    yield from arm.move(side, hold_rot(), 2.0, grip=1.0)
    yield from arm.move(at + [0, 0.005, 0], None, 1.2)
    yield from arm.set_grip(0.0, 0.8)
    ctx.flags["held"] = True
    while not ctx.flags.get("done"):
        yield
    yield from arm.set_grip(1.0, 0.6)
    yield from arm.move(side, None, 1.0)


def open_lid(ctx, peel_speed: float = 1.0):
    from yam_arena.demo import grip_rotation

    arm = ctx.right
    while not ctx.flags.get("held", ctx.flags.get("no_hold")):
        yield
    down = grip_rotation(0.0)                                    # fingers down, closed
    start = ring_point(0.0, 0.0)
    tip_offset = np.array([0, 0, PAD_TIP])                       # tool point sits PAD_TIP above the fingertips
    yield from arm.move(start + [-0.03, 0, 0.03] + tip_offset, down, 2.0, grip=0.0)
    yield from arm.move(start + [-0.012, 0, -0.003] + tip_offset, None, 1.0)   # beside the ring end, low
    yield from arm.move(start + [0.004, 0, -0.003] + tip_offset, None, 0.8)    # under it
    for a in np.linspace(0, math.radians(70), 12):                             # lift round the rivet: crack
        yield from arm.move(ring_point(a, 0.0) + [0.002, 0, -0.003] + tip_offset, None, 0.25)
    for p in np.linspace(0, math.radians(120), 16):                            # peel the lid back over the tin
        yield from arm.move(ring_point(math.radians(80), p) + [0, 0, -0.003] + tip_offset, None, 0.5 / peel_speed)
    yield from arm.move(arm.pos + [0.0, 0, 0.05], None, 1.0)
    ctx.flags["done"] = True


def script(ctx, hold: bool = True, peel_speed: float = 1.0):
    if hold:
        yield from together(hold_can(ctx), open_lid(ctx, peel_speed))
    else:
        ctx.flags["no_hold"] = True
        yield from open_lid(ctx, peel_speed)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE                                          # logging only
    return (f"tab {math.degrees(S['tab']):5.1f} cracked {S['cracked']} peel {math.degrees(S['peel']):6.1f} "
            f"tilt {math.degrees(S['tilt']):4.1f} spilled {S['spilled']} outcome {ctx.task.outcome()}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-hold", action="store_true")
    ap.add_argument("--yank", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, not a.no_hold, 4.0 if a.yank else 1.0), max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=20)
