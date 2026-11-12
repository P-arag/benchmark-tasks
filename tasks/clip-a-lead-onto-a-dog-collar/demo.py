"""Scripted full-score demo, OPEN LOOP (targets from geometry.py only; object_jitter=0).

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--no-trigger]

1. Left arm grips the snap's barrel from above, lifts it and rolls it 90 degrees so the hook stands upright (hook
   plane vertical, along x), hook pointing toward the arms (the D-ring faces them).
2. Right arm comes down on the trigger and pushes it back toward the eye (9 mm): the bolt opens.
3. Both arms move together: the gap goes up under the D-ring's bottom wire, the wire passes through it into the hook.
4. Right arm lets the trigger go (the bolt snaps shut round the wire) and lifts away; left arm opens and backs off,
   leaving the snap hanging on the D-ring.

``--no-trigger`` skips the trigger: the hook just bumps the ring (expect 0).

# CHECK (run):
#   a) BARREL_GRIP_YAW / the roll direction: after step 1 the hook's open side (bolt) must face DOWN (-z), so that
#      the wire enters from below. If it faces up, roll the other way (ROLL = -pi/2).
#   b) The D-ring swings when touched. Approach slowly (APPROACH_S); if it swings away, add a first pass where the
#      right arm's knuckle steadies the ring from behind (the dog side) before step 3.
#   c) The trigger push direction: the snap's -x maps to world +x after step 1 (eye toward +x).
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
START = np.array([g.SNAP_START[0], g.SNAP_START[1], TOP + g.SNAP_START[2]])
BARREL_C_LOCAL = np.array([-g.BARREL / 2, 0.0, 0.0])
ROLL = math.pi / 2
APPROACH_S = 2.5
WIRE = np.array([g.DRING_HINGE[0], g.DRING_HINGE[1], TOP + g.DRING_HINGE[2] - g.D_LEG - g.D_HALF])


def held(origin, local) -> np.ndarray:
    """World point of a snap-frame point once the snap is held upright (frame: x -> -x, y -> -y, z -> z)."""
    lx, ly, lz = local
    return np.asarray(origin) + np.array([-lx, -ly, lz])


def lay_world(local) -> np.ndarray:
    """World point of a snap-frame point at the start (x -> -x, y -> +z, z -> +y)."""
    lx, ly, lz = local
    return START + np.array([-lx, lz, ly])


def script(ctx, use_trigger: bool = True):
    from yam_arena.demo import grip_rotation

    L, R = ctx.left, ctx.right
    # 1. pick up the snap by its barrel and stand the hook upright
    top = grip_rotation(0.0)                                   # CHECK (run) (a): fingers must close across the barrel
    grip_pt = lay_world(BARREL_C_LOCAL)
    yield from L.move(grip_pt + [0, 0, 0.06], top, 2.0, grip=0.6)
    yield from L.move(grip_pt + [0, 0, 0.0], None, 1.0)
    yield from L.set_grip(0.0, 0.6)
    lift = grip_pt + [0, 0, 0.10]
    yield from L.move(lift, turn([1, 0, 0], ROLL) @ top, 2.0)

    # where the snap's origin must be so that the wire sits at a given snap-frame point
    def origin_for(wire_local):
        return WIRE - held(np.zeros(3), wire_local)

    below_gap = np.array([0.004, 0.0, -0.020])               # under the gap, outside the hook
    inside = np.array([0.018, 0.0, 0.000])                   # in the hook's middle
    trig_local = np.array([g.BOLT_CLOSED_TIP - g.BOLT_LEN - 0.004, 0.0, g.BOLT_Z + g.TRIGGER[2] + 0.003])

    def arm_targets(o):
        return o + held(np.zeros(3), BARREL_C_LOCAL), held(o, trig_local)

    # carry it under the ring, a little in front
    o1 = origin_for(below_gap) + [0, 0, -0.02]
    lp, tp = arm_targets(o1)
    yield from L.move(lp, None, APPROACH_S)
    if use_trigger:
        # 2. right fingertip onto the trigger, push it back toward the eye (world +x) 9 mm
        yield from R.move(tp + [0, 0, 0.05 + PAD_TIP], grip_rotation(0.0), 2.0, grip=0.0)
        yield from R.move(tp + [-0.002, 0, PAD_TIP], None, 1.0)
        yield from R.move(tp + [0.009, 0, PAD_TIP], None, 0.8)
    # 3. up through the gap, then into the hook's middle (both arms together)
    for target in (origin_for(below_gap), origin_for(np.array([0.004, 0.0, -0.004])), origin_for(inside)):
        lp, tp = arm_targets(target)
        moves = [L.move(lp, None, 1.2)]
        if use_trigger:
            moves.append(R.move(tp + [0.009, 0, PAD_TIP], None, 1.2))
        yield from together(*moves)
    # 4. let the trigger go, then the snap
    if use_trigger:
        yield from R.move(R.pos + [-0.012, 0, 0.0], None, 0.5)
        yield from R.move(R.pos + [0, 0, 0.06], None, 0.8)
    yield from L.set_grip(0.6, 0.6)
    yield from L.move(L.pos + [-0.06, 0, 0.0], None, 1.0)
    yield from wait(2.5)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE                                        # logging only
    return (f"bolt {S['bolt_open'] * 1000:4.1f} mm linked {S['linked']:.2f} inside {S['inside']:.2f} "
            f"released {S['released']} outcome {ctx.task.outcome()}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-trigger", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, not a.no_trigger), max_steps=a.max_steps, record=not a.no_record,
        log=log, log_every=20)
