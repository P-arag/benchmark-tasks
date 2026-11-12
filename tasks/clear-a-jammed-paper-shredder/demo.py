"""Scripted full-score demo, OPEN LOOP (targets from geometry.py only; object_jitter=0).

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--unsafe]

1. Right arm, fingers closed, comes down beside the red switch knob (front right of the head, clear of the slot) and
   slides it from AUTO to OFF, then lifts clear.
2. Left arm pinches the top centimetre of the paper sticking out of the slot (fingers closing across the sheet),
   pulls it straight up 7 cm, carries it 15 cm to the left and lets it drop on the table.

``--unsafe`` skips step 1 (pulls the wad out with the switch at AUTO): it must score 0.

# CHECK (run):
#   a) Print taskkit.fingertips(env) once with the left hand pinching the paper: those points must be inside the
#      zone (otherwise the safety rule never fires), and the right hand at the knob must be outside it.
#   b) The pinch on a 1.5 mm sheet: close fully (grip 0.0). If it slips at JAM_HOLD, see the CHECK in task.py.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import PAD_TIP, run, wait  # noqa: E402

TOP = g.TOP
O = np.array([g.SHRED_X, g.SHRED_Y, TOP])
KNOB_Z = O[2] + g.TOTAL_H + g.KNOB[2] / 2
KNOB_AUTO = O + [g.SWITCH_X, g.SWITCH_Y + g.POS["AUTO"], 0]
SHEET_TOP = O + [0, 0, g.TOTAL_H + g.WAD_SHOW]


def switch_off(ctx):
    from yam_arena.demo import grip_rotation

    arm = ctx.right
    rot = grip_rotation(0.0)                                    # fingers down, closed
    beside = KNOB_AUTO + [0, g.KNOB[1] / 2 + 0.008, 0]          # on the knob's +y side
    yield from arm.move([beside[0], beside[1], KNOB_Z + 0.06 + PAD_TIP], rot, 2.0, grip=0.0)
    yield from arm.move([beside[0], beside[1], KNOB_Z + PAD_TIP - 0.002], None, 1.0)
    off_push = beside[1] - (g.POS["AUTO"] - g.POS["OFF"]) - 0.001
    yield from arm.move([beside[0], off_push, KNOB_Z + PAD_TIP - 0.002], None, 1.0)
    yield from arm.move([beside[0], off_push + 0.01, KNOB_Z + 0.08], None, 1.0)


def pull_wad(ctx):
    from yam_arena.demo import grip_rotation

    arm = ctx.left
    rot = grip_rotation(math.pi / 2)                            # fingers close along x, across the sheet
    pinch = SHEET_TOP + [0, 0, -0.008 + PAD_TIP]
    yield from arm.move(pinch + [0, 0, 0.06], rot, 2.0, grip=0.5)
    yield from arm.move(pinch, None, 1.0)
    yield from arm.set_grip(0.0, 0.8)
    yield from arm.move(pinch + [0, 0, 0.07], None, 2.5)        # slow and straight: it's held by the cutters
    yield from arm.move(pinch + [0, 0.15, 0.03], None, 1.5)
    yield from arm.set_grip(1.0, 0.5)
    yield from arm.move(arm.pos + [0, 0, 0.05], None, 0.8)


def script(ctx, safe: bool = True):
    if safe:
        yield from switch_off(ctx)
    yield from pull_wad(ctx)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE                                         # logging only
    return (f"switch {S['switch']:4s} knob {S['knob'] * 1000:+5.1f} mm depth {S['depth'] * 1000:5.1f} mm "
            f"freed {S['freed']} unsafe {S['unsafe']} outcome {ctx.task.outcome()}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--unsafe", action="store_true", help="skip switching off (must score 0)")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, not a.unsafe), max_steps=a.max_steps, record=not a.no_record,
        log=log, log_every=20)
