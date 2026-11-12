"""On the station table: a desktop thermal receipt printer, front toward the arms, its lid shut and latched over an
empty paper bucket, and a new 80 mm paper roll lying on its side to the printer's right. The instruction only asks
for the roll to be loaded so the printer can print: that means pressing the lid release, dropping the roll in the
right way round (the sticker in the bucket shows the paper leaving the top of the roll toward the front), leaving
its free end out past the tear bar, and pressing the lid shut until it latches.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Load the new paper roll so the printer can print." --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, on_reset, on_step, write_joint_positions  # noqa: E402

import geometry as g  # noqa: E402
import taskkit as tk  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Load the new paper roll so the printer can print."
TOP = g.TOP

# -- objects --------------------------------------------------------------------------------------------------------
PRINTER = UsdBody("receipt_printer", usd="assets/receipt_printer.usda", pos=(g.PRINTER_X, g.PRINTER_Y, TOP),
                  size=(g.D, g.W, g.H), articulation=True,
                  joint_pos=(("lid_hinge", 0.0), ("button_press", 0.0)),
                  drives=(("lid_hinge", g.POP_K, 0.05), ("button_press", g.BUTTON_K, 2.0)),
                  friction=(0.6, 0.5))
ROLL = UsdBody("paper_roll", usd="assets/paper_roll.usda",
               pos=(g.ROLL_START[0], g.ROLL_START[1], TOP + g.ROLL_START[2]), size=(2 * g.ROLL_R, g.ROLL_W, 2 * g.ROLL_R),
               articulation=True, joint_pos=tuple((f"tail_{k:02d}", 0.0) for k in range(g.TAIL_N)),
               drives=((r"tail_\d\d", g.TAIL_BEND_K, g.TAIL_BEND_D),), friction=(0.7, 0.6), solver_iters=32)
# CHECK (run): the tail starts straight out behind the roll's top and must flop down onto the table while the
# scene settles at reset. Look at the first frame of the top camera: the free end should lie flat on the table about
# 5-8 cm behind the roll. Write the measured spot into README/setup ("its free end lies ... behind the roll").
OBJECTS = [PRINTER, ROLL]

# -- monitor: the lid latch, the pop-up spring and stillness ---------------------------------------------------------
_STATE: dict = {}
_STILL = tk.Still()


def _latch(environment) -> None:
    tk.write_limits(environment, "receipt_printer", "lid_hinge", 0.0, 0.0)


def _unlatch(environment) -> None:
    tk.write_limits(environment, "receipt_printer", "lid_hinge", 0.0, g.LID_MAX)


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(latched=True, armed=False, lid=0.0, depth=0.0, roll_still=0.0, why=[])
    _STILL.reset()
    if environment is not None:
        _latch(environment)


@before_reset
def _restart(environment) -> None:
    _latch(environment)
    write_joint_positions(environment, "receipt_printer", {"lid_hinge": 0.0, "button_press": 0.0})
    write_joint_positions(environment, "paper_roll", {f"tail_{k:02d}": 0.0 for k in range(g.TAIL_N)})


_reset_state()


@on_step
def _monitor(environment) -> None:
    from scenekit import set_joint_targets
    from yam_arena.scene import object_states

    st = object_states(environment, ["receipt_printer", "paper_roll"])
    joints = st["receipt_printer"].joints
    lid = _STATE["lid"] = float(joints["lid_hinge"])
    _STATE["depth"] = -float(joints["button_press"])
    if _STATE["latched"]:
        if _STATE["depth"] > g.BUTTON_PRESSED:                 # release pressed: the latch lets go
            _STATE["latched"], _STATE["armed"] = False, False
            _unlatch(environment)
    else:
        if lid > g.REARM_ABOVE:
            _STATE["armed"] = True
        if _STATE["armed"] and lid < g.LATCH_BELOW and _STATE["depth"] < g.BUTTON_PRESSED:
            _STATE["latched"] = True                           # pressed shut: click
            _latch(environment)
            write_joint_positions(environment, "receipt_printer", {"lid_hinge": 0.0})
    # the pop-up spring only acts over the first POP_ANGLE: beyond that its target follows the lid (no force)
    set_joint_targets(environment, "receipt_printer", {"lid_hinge": max(g.POP_ANGLE, lid) if not _STATE["latched"]
                                                       else 0.0})
    r = st["paper_roll"]
    _STATE["roll_still"] = _STILL.update("roll", r.pos, r.quat_xyzw, r.lin_vel, r.ang_vel)
    _STATE["geom"] = _roll_geometry(environment, r)


def _roll_geometry(environment, roll) -> dict:
    """Where the roll and its free end are, in the printer's frame, and which way the paper is wound."""
    links = tk.body_poses(environment, "paper_roll")
    origin = np.array([g.PRINTER_X, g.PRINTER_Y, TOP])
    centre = np.asarray(roll.pos) - origin
    root = links["tail_00"][0] - origin
    nxt = links["tail_01"][0] - origin
    tip = links[f"tail_{g.TAIL_N - 1:02d}"][0] - origin
    axis = tk.rotation(roll.quat_xyzw)[:, 1]                    # the roll's own axis (local y)
    r, t = root - centre, nxt - root
    sense = int(np.sign(np.cross(r, t)[1])) if abs(np.cross(r, t)[1]) > 1e-6 else 0
    return {"centre": centre, "tip": tip, "axis": axis, "sense": sense,
            "tip_in_slot_band": bool(g.HEAD_TOP - 0.03 < tip[2] < g.H + 0.01)}


# -- scoring ---------------------------------------------------------------------------------------------------------
def in_bucket() -> bool:
    geo = _STATE.get("geom")
    if not geo:
        return False
    c = geo["centre"]
    off = math.hypot(c[0] - g.BUCKET_X, c[2] - g.BUCKET_Z)
    axis_err = math.acos(min(1.0, abs(float(geo["axis"][1]))))
    return off <= g.IN_BUCKET_TOL and axis_err <= g.AXIS_TOL and _STATE["roll_still"] >= g.STILL_S


def right_way_round() -> bool:
    geo = _STATE.get("geom")
    # the sign is taken against the printer's +y; a roll turned end for end (axis along -y) winds the other way in
    # its own frame but the world-frame cross product already accounts for that
    return bool(geo and geo["sense"] == g.CORRECT_SENSE)


def end_out() -> bool:
    """The free end hangs out of the front at least 2 cm past the tear bar (between platen and head)."""
    geo = _STATE.get("geom")
    return bool(geo and geo["tip"][0] <= g.TEAR_X - g.OUT_PAST_TEAR)
    # CHECK (run): a tip that is 2 cm in front of the printer but went round the OUTSIDE of the lid (over the top,
    # not through the slot) would pass this test. With the lid latched that is only possible if the strip lies over
    # the lid; add a test that some tail link is inside the slot band (tip_in_slot_band / link x within 1 cm of
    # TEAR_X and z between HEAD_TOP and HEAD_TOP + SLOT + 2 mm) if that loophole shows up in Dream-1 runs.


def outcome(objects=None) -> str:
    """'done', 'end_inside' (roll right, lid latched, free end shut inside), 'lid_open' (roll right, lid not
    latched), 'reversed' (roll in the bucket the wrong way round), or 'other'."""
    if not in_bucket():
        return "other"
    if not right_way_round():
        return "reversed"
    if not _STATE["latched"]:
        return "lid_open"
    return "done" if end_out() else "end_inside"


def success(objects) -> bool:
    """Roll in the bucket the right way round, free end out past the tear bar, lid latched; held 1 s."""
    return outcome() == "done"


def score(objects) -> float:
    """1.0 done; 0.5 lid latched over a correctly loaded roll whose free end is shut inside (the printer can't feed
    until the lid is opened again); 0.25 roll loaded the right way round but the lid not latched; 0 otherwise,
    including a roll loaded backwards (it prints nothing on thermal paper) or not in the bucket."""
    return {"done": 1.0, "end_inside": 0.5, "lid_open": 0.25}.get(outcome(), 0.0)
