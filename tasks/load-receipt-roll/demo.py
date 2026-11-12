"""Scripted full-score demo, OPEN LOOP: every target below comes from geometry.py (the fixed setup) or from
constants a person reads off the top-camera image, never from simulator state. Run it with object_jitter=0.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--no-turn]

1. Right arm presses the lid release; the lid pops up ~11 degrees.
2. Left arm hooks the lid's front lip from below and swings it open past vertical onto its stop.
3. Right arm picks the roll from above (fingers on its two flat ends), turns it 180 degrees about vertical so its
   free end leaves the top toward the FRONT, and lowers it into the bucket.
4. Right arm pinches the free end where it lies over the head and draws it 4 cm out over the tear bar.
5. Left arm swings the lid back over and presses its front edge down until it latches.

``--no-turn`` drops the roll in as it lies (wrong way round): expect 0 ("reversed").

# CHECK (run): this script has never been run. Expect to tune, in order:
#   a) LID_HOOK and the swing arc (the lip is only 6 mm thick: come in 5 mm below it),
#   b) ROLL_GRIP_Z and the wrist's 180-degree turn (split it into two 90-degree turns with a regrasp if the wrist
#      runs out of range; demokit reports worst IK error in result.json),
#   c) TAIL_PICK: where the free end lies after the drop. Read it off the top camera's frame at the end of step 3
#      in a dry run and write it here. If it varies between runs, record a teleoperated demo instead (the rules
#      allow "a scripted or teleoperated demo").
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import PAD_TIP, run, turn, wait  # noqa: E402

TOP = g.TOP
P = np.array([g.PRINTER_X, g.PRINTER_Y, TOP])            # the printer's origin, world frame


def world(local) -> np.ndarray:
    return P + np.asarray(local, float)


BUTTON = world((g.BUTTON_X, g.BUTTON_Y, g.H + g.BUTTON_H))
LID_FRONT = world((g.HINGE_X - g.LID_L, 0.0, g.H))       # front edge of the shut lid
LID_HOOK = LID_FRONT + [-0.004, 0.0, -0.006]              # fingertips just under the popped lip
ROLL_C = np.array([g.ROLL_START[0], g.ROLL_START[1], TOP + g.ROLL_START[2]])
ROLL_GRIP_Z = ROLL_C[2] + 0.010                           # tool point a little above the axis (sim)
BUCKET = world((g.BUCKET_X, 0.0, g.BUCKET_Z))
TAIL_PICK = world((g.FRONT_X + 0.012, 0.0, g.HEAD_TOP + 0.004))   # CHECK (run): measure, see (c) above
TRAVEL_Z = TOP + 0.19                                     # straight-down grips reach about 20 cm over the table


def top_grip(yaw: float) -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return grip_rotation(yaw)


def press_release(ctx):
    arm = ctx.right
    yield from arm.move(BUTTON + [0, 0, 0.05], top_grip(0.0), 2.0, grip=0.0)
    yield from arm.move(BUTTON + [0, 0, -0.006], None, 0.8)          # 6 mm past its top: 4 mm travel + wrist give
    yield from wait(0.5)
    yield from arm.move(BUTTON + [0, 0, 0.06], None, 0.8)


def swing_lid(arm, a_from: float, a_to: float, seconds: float, steps: int = 12):
    """Carry the lid's front edge round its hinge from angle a_from to a_to, fingertips under the lip."""
    hinge = world((g.HINGE_X, 0.0, g.HINGE_Z))
    for k in range(1, steps + 1):
        a = a_from + (a_to - a_from) * k / steps
        edge = hinge + g.LID_L * np.array([-math.cos(a), 0.0, math.sin(a)])
        yield from arm.move(edge + [-0.006, 0, -0.006], None, seconds / steps)


def open_lid(ctx):
    arm = ctx.left
    rot = turn([0, 1, 0], math.radians(60)) @ top_grip(math.pi / 2)  # fingers pointing forward-down, closed
    pop = math.radians(10)
    hinge = world((g.HINGE_X, 0.0, g.HINGE_Z))
    lip = hinge + g.LID_L * np.array([-math.cos(pop), 0.0, math.sin(pop)])
    yield from arm.move(lip + [-0.05, 0, 0.0], rot, 2.0, grip=0.0)
    yield from arm.move(lip + [-0.004, 0, -0.006], None, 1.0)
    yield from swing_lid(arm, pop, math.radians(100), 3.0)
    yield from arm.move(arm.pos + [-0.06, 0, 0.03], None, 1.0)     # let go; the lid rests back on its stop


def load_roll(ctx, do_turn: bool = True):
    arm = ctx.right
    yaw0 = 0.0                                                       # fingers close along y, onto the flat ends
    yield from arm.move(ROLL_C + [0, 0, 0.10], top_grip(yaw0), 2.0, grip=1.0)
    yield from arm.move([ROLL_C[0], ROLL_C[1], ROLL_GRIP_Z], None, 1.0)
    yield from arm.set_grip(0.0, 0.8)
    yield from arm.move(ROLL_C + [0, 0, 0.12], None, 1.0)
    above = BUCKET + [0, 0, 0.09]
    yaw1 = yaw0 + (math.pi if do_turn else 0.0)
    yield from arm.move(above, top_grip(yaw1), 3.0)                  # carry and turn end for end on the way
    yield from arm.move(BUCKET + [0, 0, 0.012], None, 1.2)           # just above its seat
    yield from arm.set_grip(1.0, 0.6)
    yield from arm.move(above, None, 1.0)


def draw_end_out(ctx):
    arm = ctx.right
    rot = top_grip(math.pi / 2)                                      # fingers close along x, pinching the strip
    yield from arm.move(TAIL_PICK + [0, 0, 0.05], rot, 1.5, grip=0.4)
    yield from arm.move(TAIL_PICK + [0, 0, PAD_TIP - 0.002], None, 1.0)
    yield from arm.set_grip(0.0, 0.6)
    yield from arm.move(TAIL_PICK + [-0.045, 0, PAD_TIP - 0.010], None, 1.5)   # over the tear bar, a little down
    yield from arm.set_grip(1.0, 0.5)
    yield from arm.move(arm.pos + [-0.05, 0, 0.06], None, 1.0)


def close_lid(ctx):
    arm = ctx.left
    hinge = world((g.HINGE_X, 0.0, g.HINGE_Z))
    rot = turn([0, 1, 0], math.radians(-30)) @ top_grip(math.pi / 2)
    top_of_open = hinge + g.LID_L * np.array([-math.cos(math.radians(100)), 0, math.sin(math.radians(100))])
    yield from arm.move(top_of_open + [0.01, 0, 0.02], rot, 2.0, grip=0.0)   # behind the lid's raised front edge
    # push it forward over the top and let it fall onto the pop-up spring, then press the front edge down
    for a in np.linspace(math.radians(100), math.radians(15), 10):
        edge = hinge + g.LID_L * np.array([-math.cos(a), 0, math.sin(a)])
        yield from arm.move(edge + [0.008, 0, 0.008], None, 0.35)
    yield from arm.move(LID_FRONT + [0.015, 0, 0.002], top_grip(math.pi / 2), 1.0)
    yield from arm.move(LID_FRONT + [0.015, 0, -0.004], None, 0.8)          # press shut past the latch
    yield from wait(0.4)
    yield from arm.move(LID_FRONT + [0.015, 0, 0.08], None, 1.0)


def script(ctx, do_turn: bool = True):
    yield from press_release(ctx)
    yield from open_lid(ctx)
    yield from load_roll(ctx, do_turn)
    yield from draw_end_out(ctx)
    yield from close_lid(ctx)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE                                              # logging only; never used to steer the arms
    geo = S.get("geom") or {}
    return (f"lid {math.degrees(S['lid']):5.1f} deg latched {S['latched']} button {S['depth'] * 1000:3.1f} mm  "
            f"sense {geo.get('sense')} tip_x {geo.get('tip', [np.nan])[0]:+.3f}  outcome {ctx.task.outcome()}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-turn", action="store_true", help="drop the roll in as it lies (wrong way round)")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, do_turn=not a.no_turn), max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=20)
