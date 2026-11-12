"""Scripted full-score demo, OPEN LOOP (targets from geometry.py and the setup only; object_jitter=0).

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--no-turn]

1. Left arm pinches the chain's top run 4 cm behind the bottom-bracket shell, lifts it 1 cm, moves it 8 mm outboard
   into the chainring's plane and lays it over the ring's top-rear teeth (about 11 o'clock seen from the arms).
2. Right arm grips the pedal from the front and carries it round the crank's circle, forward (the ring's top moving
   toward the front of the bike, drawing the chain on), while the left arm keeps the chain pressed onto the teeth for
   the first quarter turn, then lets go and moves clear.
3. The right arm carries on for 1.5 more revolutions so the crank turns a full turn with the chain on, then lets go.

``--no-turn`` stops after step 1: expect 0.25 or 0.5, never 1.0.

# CHECK (run):
#   a) TOP_RUN_PICK: where the slack top run hangs after reset settles (read off the top camera / setup.png).
#   b) The pedal's lowest point is 3.5 cm above the table: the gripper must not hit the table. If it does, grip
#      the pedal from above in the top half of the circle only, and turn the lower half with a second regrasp.
#   c) Direction: if the chain climbs off instead of on, run the crank the other way (DIRECTION = -1).
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, together, turn, wait  # noqa: E402

TOP = g.TOP
AXLE = np.array([g.RING_X, g.RING_Y, TOP + g.AXLE_Z])
TOP_RUN_PICK = np.array([g.DROP_X, g.RING_Y - 0.04, TOP + g.AXLE_Z + 0.022])   # CHECK (run): see (a)
LAY_AT = math.radians(110)          # where on the ring to lay the chain, angle from +y in the yz plane
DIRECTION = +1                      # crank rotation sense about +x


def pedal_at(theta: float) -> np.ndarray:
    """The pedal's centre for crank angle theta (0 = arm straight down, + about world x)."""
    return np.array([g.PEDAL_X, AXLE[1] + g.CRANK_LEN * math.sin(theta), AXLE[2] - g.CRANK_LEN * math.cos(theta)])


def front_grip() -> np.ndarray:
    """Fingers pointing +x (horizontal, from the arms' side), closing along z, onto the pedal's front edge."""
    from yam_arena.demo import grip_rotation

    return turn([0, 1, 0], math.pi / 2) @ grip_rotation(0.0)


def lay_chain(ctx):
    from yam_arena.demo import grip_rotation

    arm = ctx.left
    rot = grip_rotation(math.pi / 2)                           # top grip, fingers closing along y across the run
    yield from arm.move(TOP_RUN_PICK + [0, 0, 0.06], rot, 2.0, grip=1.0)
    yield from arm.move(TOP_RUN_PICK, None, 1.0)
    yield from arm.set_grip(0.0, 0.6)
    yield from arm.move(TOP_RUN_PICK + [0, 0, 0.012], None, 0.6)
    on_teeth = AXLE + (g.R_RING + 0.004) * np.array([0, math.cos(LAY_AT), math.sin(LAY_AT)])
    on_teeth[0] = g.RING_X
    yield from arm.move(on_teeth + [0, 0, 0.012], None, 1.5)    # over the teeth, in the ring's plane
    yield from arm.move(on_teeth, None, 0.8)                   # press the rollers down between them


def turn_crank(ctx, revolutions: float, release_left_after: float):
    arm = ctx.right
    rot = front_grip()
    th0 = g.CRANK_START
    p0 = pedal_at(th0)
    yield from arm.move(p0 + [-0.06, 0, 0], rot, 2.0, grip=1.0)
    yield from arm.move(p0 + [-0.012, 0, 0], None, 1.0)
    yield from arm.set_grip(0.0, 0.6)
    n = int(48 * revolutions)
    for k in range(1, n + 1):
        th = th0 + DIRECTION * 2 * math.pi * revolutions * k / n
        yield from arm.move(pedal_at(th) + [-0.012, 0, 0], None, 0.25)    # ~12 s per revolution
        if k == int(48 * release_left_after):
            ctx.flags["release"] = True
    yield from arm.set_grip(1.0, 0.5)
    yield from arm.move(arm.pos + [-0.08, 0, 0.04], None, 1.0)


def hold_then_release(ctx):
    arm = ctx.left
    while not ctx.flags.get("release"):
        yield
    yield from arm.set_grip(1.0, 0.4)
    yield from arm.move(arm.pos + [-0.06, 0, 0.06], None, 1.0)


def script(ctx, do_turn: bool = True):
    yield from lay_chain(ctx)
    if do_turn:
        yield from together(turn_crank(ctx, 2.0, release_left_after=0.25), hold_then_release(ctx))
    else:
        yield from ctx.left.set_grip(1.0, 0.4)
        yield from ctx.left.move(ctx.left.pos + [-0.06, 0, 0.06], None, 1.0)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE                                        # logging only
    return (f"on_ring {S['on_ring']:2d} on_cog {S['on_cog']:2d} crank {math.degrees(S['crank']):7.1f} "
            f"turned {math.degrees(S['turned']):6.1f} outcome {ctx.task.outcome()}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-turn", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, not a.no_turn), max_steps=a.max_steps, record=not a.no_record,
        log=log, log_every=20)
