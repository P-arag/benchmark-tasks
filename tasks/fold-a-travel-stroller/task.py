"""On the station table: a compact travel stroller standing open on its four wheels, push handles toward the arms.
The instruction: "Fold the stroller." Its fold is locked until both red release buttons, one each side at the main
hinge, are pressed in at the same time (a child-safety two-button release); then the front legs swing back onto the
rear legs and a storage latch catches it folded.

!!! Run reach_check.py first: at real size the buttons may be out of the YAM's reach (see geometry.py).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Fold the stroller." --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, on_reset, on_step, write_joint_positions  # noqa: E402

import geometry as g  # noqa: E402
import taskkit as tk  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Fold the stroller."
TOP = g.TOP

STROLLER = UsdBody("stroller", usd="assets/stroller.usda", pos=(g.ORIGIN_X, g.ORIGIN_Y, TOP),
                   size=(0.75, 2 * g.HALF_W, g.HANDLE_TOP[1]), articulation=True,
                   joint_pos=(("fold", 0.0), ("press_l", 0.0), ("press_r", 0.0)),
                   drives=(("fold", 0.0, g.FOLD_DAMPING), ("press_l", g.BUTTON_K, 2.0), ("press_r", g.BUTTON_K, 2.0)),
                   friction=g.WHEEL_FRICTION, solver_iters=32)
OBJECTS = [STROLLER]

_STATE: dict = {}
_STILL = tk.Still()


def _lock(environment, at: float) -> None:
    tk.write_limits(environment, "stroller", "fold", at, at)


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(unlocked=False, ever_unlocked=False, latched=False, fold=0.0, pressed=(0.0, 0.0), tilt=0.0,
                  tipped=False, still=0.0, why=[])
    _STILL.reset()
    if environment is not None:
        _lock(environment, 0.0)


@before_reset
def _restart(environment) -> None:
    _lock(environment, 0.0)
    write_joint_positions(environment, "stroller", {"fold": 0.0, "press_l": 0.0, "press_r": 0.0})


_reset_state()


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    s = object_states(environment, ["stroller"])["stroller"]
    fold = _STATE["fold"] = float(s.joints["fold"])
    left, right = -float(s.joints["press_l"]), float(s.joints["press_r"])     # both positive when pressed in
    _STATE["pressed"] = (left, right)
    both = left > g.BUTTON_PRESSED and right > g.BUTTON_PRESSED
    if not _STATE["latched"]:
        if both and not _STATE["unlocked"]:
            _STATE["unlocked"] = _STATE["ever_unlocked"] = True
            tk.write_limits(environment, "stroller", "fold", 0.0, g.FOLD_MAX)
        elif _STATE["unlocked"] and not both and fold < g.UNLOCK_KEEP:
            _STATE["unlocked"] = False                                       # let go near open: it re-locks open
            _lock(environment, 0.0)
            write_joint_positions(environment, "stroller", {"fold": 0.0})
        if _STATE["unlocked"] and fold >= g.FOLD_LATCH:
            _STATE["latched"] = True                                         # click: storage latch
            _lock(environment, fold)
    # CHECK (run): the re-lock snaps the fold back to 0 if it was opened less than 8 degrees; watch that this doesn't
    # jolt the stroller. A gentler version locks at the current angle and lets the robot open it fully.
    _STATE["tilt"] = tk.tilt(s.quat_xyzw)
    if _STATE["tilt"] > g.TIPPED and not _STATE["tipped"]:
        _STATE["tipped"] = True
        _STATE["why"].append("stroller tipped over")
    _STATE["still"] = _STILL.update("stroller", s.pos, s.quat_xyzw, s.lin_vel, s.ang_vel)


def outcome(objects=None) -> str:
    """'folded' (latched folded, standing within 30 degrees of upright, still), 'half' (folded >= 20 degrees, not
    latched), 'unlocked' (both buttons pressed together at some point, fold < 20 degrees), or 'other' (including a
    stroller tipped over at any time)."""
    if _STATE["tipped"]:
        return "other"
    if _STATE["latched"] and _STATE["tilt"] <= g.STAND_TILT and _STATE["still"] >= g.STILL_S:
        return "folded"
    if _STATE["fold"] >= g.HALF_FOLD:
        return "half"
    if _STATE["ever_unlocked"]:
        return "unlocked"
    return "other"


def success(objects) -> bool:
    return outcome() == "folded"


def score(objects) -> float:
    """1.0 folded and latched, standing; 0.5 folded at least 20 degrees but not latched; 0.25 released (both buttons
    together) but not folded 20 degrees; 0 anything else, or if the stroller ever tipped over (past 60 degrees)."""
    return {"folded": 1.0, "half": 0.5, "unlocked": 0.25}.get(outcome(), 0.0)
