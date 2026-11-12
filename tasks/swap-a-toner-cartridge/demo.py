"""Scripted full-score demo, OPEN LOOP (targets from geometry.py only; object_jitter=0).

    python demo.py task.py demo_out [--max-steps 3600] [--no-record] [--keep-cover]

1. Left arm hooks the door's top lip and swings the door down open (90 degrees).
2. Right arm grips the old cartridge's handle, draws it straight out 14 cm (past the detent and clear of the
   cavity), carries it left and lowers it into the return box.
3. Right arm lifts the orange cover off the new cartridge by its tab and drops it on the table.
4. Right arm grips the new cartridge's handle, carries it to the cavity mouth, pushes it in until home (5 mm past
   the detent's click, held by the back wall).
5. Left arm swings the door back up and presses it shut.

``--keep-cover`` skips step 3: the cartridge jams 12 mm short of home (expect 0).

# CHECK (run):
#   a) The front grip on the handle (fingers horizontal, pointing +x, closing along z). The handle is only 15 mm
#      thick; open the gripper to ~40%.
#   b) Carrying a 30 cm cartridge by a 8 cm handle: it may swing; slow the carry (CARRY_S) if it does.
#   c) The run may need more than 2,400 steps; README's run limit says 3,600 (3 minutes). Measure the demo's length
#      and set run_limit to about 1.5x it.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, turn, wait  # noqa: E402

TOP = g.TOP
P = np.array([g.PRINTER_X, g.PRINTER_Y, TOP])
FRONT_X = P[0] - g.D / 2
SEAT = P + [g.SEAT_X, 0, g.SEAT_Z]
NEW = np.array([g.NEW_XY[0], g.NEW_XY[1], TOP + 0.010 + g.C_H / 2])
BOX = np.array([g.BOX_XY[0], g.BOX_XY[1], TOP])
HINGE = np.array([FRONT_X, P[1], TOP + g.DOOR_HINGE_Z])
CARRY_S = 2.5


def handle_of(centre) -> np.ndarray:
    """The tool point on a cartridge's handle bar (centre of the bar)."""
    return np.asarray(centre) + [-g.C_D / 2 - g.HANDLE_OUT + g.HANDLE[0] / 2, 0, 0.010]


def front_grip() -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return turn([0, 1, 0], math.pi / 2) @ grip_rotation(0.0)


def door_top(angle: float) -> np.ndarray:
    return HINGE + g.DOOR_H * np.array([math.sin(angle), 0, math.cos(angle)])


def open_door(ctx):
    from yam_arena.demo import grip_rotation

    arm = ctx.left
    rot = turn([0, 1, 0], math.radians(45)) @ grip_rotation(math.pi / 2)
    yield from arm.move(door_top(0) + [-0.04, 0, 0.03], rot, 2.0, grip=0.0)
    yield from arm.move(door_top(0) + [-0.012, 0, -0.002], None, 1.0)          # fingertips behind the lip
    for a in np.linspace(0, g.DOOR_OPEN, 10):
        yield from arm.move(door_top(a) + [-0.012, 0, -0.002], None, 0.3)
    yield from arm.move(arm.pos + [-0.05, 0, 0.06], None, 1.0)


def pull_old(ctx):
    arm = ctx.right
    h = handle_of(SEAT)
    yield from arm.move(h + [-0.08, 0, 0.0], front_grip(), 2.5, grip=0.45)
    yield from arm.move(h, None, 1.2)
    yield from arm.set_grip(0.0, 0.8)
    yield from arm.move(h + [-0.14, 0, 0.003], None, 2.0)                     # straight out, past the detent
    yield from arm.move(h + [-0.14, 0, 0.06], None, 1.0)
    over_box = BOX + [-g.C_D / 2 - g.HANDLE_OUT, 0, g.BOX_IN[2] + g.C_H / 2 + 0.03]
    yield from arm.move(over_box, None, CARRY_S)
    yield from arm.move(over_box + [0, 0, -0.03 - g.BOX_IN[2] + 0.02], None, 1.5)
    yield from arm.set_grip(0.45, 0.6)
    yield from arm.move(arm.pos + [-0.06, 0, 0.0], None, 0.8)
    yield from arm.move(arm.pos + [0, 0, 0.10], None, 0.8)


def remove_cover(ctx):
    from yam_arena.demo import grip_rotation

    arm = ctx.right
    tab = NEW + [-g.C_D / 2 + 0.015, 0, g.C_H / 2 + g.COVER_TOP + 0.010]
    yield from arm.move(tab + [0, 0, 0.06], grip_rotation(0.0), 2.0, grip=0.6)  # fingers close along x on the tab
    yield from arm.move(tab, None, 1.0)
    yield from arm.set_grip(0.0, 0.6)
    yield from arm.move(tab + [0.02, 0, 0.05], None, 1.0)                      # up and back, off the lip
    yield from arm.move(np.array([0.20, -0.10, TOP + 0.06]), None, 1.5)
    yield from arm.set_grip(1.0, 0.5)


def insert_new(ctx):
    arm = ctx.right
    h = handle_of(NEW)
    yield from arm.move(h + [-0.08, 0, 0.0], front_grip(), 2.0, grip=0.45)
    yield from arm.move(h, None, 1.2)
    yield from arm.set_grip(0.0, 0.8)
    yield from arm.move(h + [0, 0, 0.05], None, 1.0)
    mouth = handle_of(SEAT) + [-0.14, 0, 0.004]
    yield from arm.move(mouth + [0, 0, 0.03], None, CARRY_S)
    yield from arm.move(mouth, None, 1.0)
    yield from arm.move(handle_of(SEAT) + [0.004, 0, 0.002], None, 3.0)       # slide home, a little past the click
    yield from arm.set_grip(0.45, 0.6)
    yield from arm.move(arm.pos + [-0.08, 0, 0], None, 1.0)


def close_door(ctx):
    from yam_arena.demo import grip_rotation

    arm = ctx.left
    rot = turn([0, 1, 0], math.radians(-30)) @ grip_rotation(math.pi / 2)
    under = door_top(g.DOOR_OPEN) + [0.0, 0, -0.015]
    yield from arm.move(under + [-0.03, 0, -0.02], rot, 2.0, grip=0.0)
    for a in np.linspace(g.DOOR_OPEN, -math.radians(8), 10):
        yield from arm.move(door_top(a) + [-0.010, 0, -0.010], None, 0.35)
    yield from arm.move(door_top(0) + [-0.004, 0, -0.02], None, 0.8)          # press it into the magnet
    yield from arm.move(door_top(0) + [-0.06, 0, 0.0], None, 0.8)


def script(ctx, cover_off: bool = True):
    yield from open_door(ctx)
    yield from pull_old(ctx)
    if cover_off:
        yield from remove_cover(ctx)
    yield from insert_new(ctx)
    yield from close_door(ctx)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE                                                        # logging only
    return f"door {math.degrees(S['door']):6.1f} dropped {S['dropped']} outcome {ctx.task.outcome()}"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=3600)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--keep-cover", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, not a.keep_cover), max_steps=a.max_steps, record=not a.no_record,
        log=log, log_every=20)
