"""On the station table: a full 400 g tin of tomato soup with a ring-pull (full-aperture easy-open) lid, standing
free in front of the arms, its ring toward them. The instruction is only "Open the can." Opening it takes lifting
the ring until the score cracks, then peeling the lid back off the opening, without spilling: once it is open, the
soup 1 cm below the rim spills if the tin tilts past about 15 degrees. The tin isn't fixed, so the other arm should
steady it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Open the can." --scorer task_score ...
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
INSTRUCTION = "Open the can."
TOP = g.TOP

CAN = UsdBody("soup_can", usd="assets/soup_can.usda", pos=(g.CAN_X, g.CAN_Y, TOP),
              size=(2 * g.CAN_R, 2 * g.CAN_R, g.CAN_H), articulation=True,
              joint_pos=(("lid_peel", 0.0), ("tab_lever", 0.0)),
              drives=(("tab_lever", g.TAB_K, g.TAB_D), ("lid_peel", 0.0, g.PEEL_DAMPING)),
              friction=(0.6, 0.5), solver_iters=32, torsional_patch=0.01)
OBJECTS = [CAN]

_STATE: dict = {}
_STILL = tk.Still()


def _lock_lid(environment) -> None:
    tk.write_limits(environment, "soup_can", "lid_peel", 0.0, 0.0)


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(cracked=False, tab=0.0, peel=0.0, best_peel=0.0, tilt=0.0, spilled=False, knocked=False,
                  still=0.0, why=[])
    _STILL.reset()
    if environment is not None:
        _lock_lid(environment)


@before_reset
def _restart(environment) -> None:
    _lock_lid(environment)
    write_joint_positions(environment, "soup_can", {"lid_peel": 0.0, "tab_lever": 0.0})


_reset_state()


@on_step
def _monitor(environment) -> None:
    from scenekit import set_joint_targets
    from yam_arena.scene import object_states

    s = object_states(environment, ["soup_can"])["soup_can"]
    tab = _STATE["tab"] = float(s.joints["tab_lever"])
    peel = _STATE["peel"] = float(s.joints["lid_peel"])
    _STATE["best_peel"] = max(_STATE["best_peel"], peel)
    if not _STATE["cracked"] and tab >= g.CRACK_ANGLE:
        _STATE["cracked"] = True                               # pop: the score is broken, the lid can be peeled
        tk.write_limits(environment, "soup_can", "lid_peel", 0.0, g.PEEL_MAX)
    # before the crack the tab springs back toward flat; after it, the bent tab and torn lid stay where they are put
    if _STATE["cracked"]:
        set_joint_targets(environment, "soup_can", {"tab_lever": tab, "lid_peel": peel})
    _STATE["tilt"] = tk.tilt(s.quat_xyzw)
    if _STATE["tilt"] > math.radians(60):
        _STATE["knocked"] = True
    if _STATE["cracked"] and _STATE["tilt"] > g.SPILL_TILT and not _STATE["spilled"]:
        _STATE["spilled"] = True
        _STATE["why"].append(f"spilled: tilted {math.degrees(_STATE['tilt']):.0f} deg after opening")
    _STATE["still"] = _STILL.update("can", s.pos, s.quat_xyzw, s.lin_vel, s.ang_vel)


def outcome(objects=None) -> str:
    """'open' (lid folded back >= 100 degrees, no spill), 'half' (lid peeled >= 45 degrees), 'cracked', or 'other'
    (including any spill after the crack, and a tin knocked over at any time)."""
    if _STATE["spilled"] or _STATE["knocked"] or not _STATE["cracked"]:
        return "other"
    if _STATE["peel"] >= g.OPEN_ANGLE and _STATE["still"] >= g.STILL_S:
        return "open"
    if _STATE["peel"] >= g.HALF_OPEN:
        return "half"
    return "cracked"


def success(objects) -> bool:
    return outcome() == "open"


def score(objects) -> float:
    """1.0 lid folded back clear of the opening with nothing spilled; 0.5 the lid peeled back at least 45 degrees;
    0.25 the seal cracked but the lid not peeled; 0 nothing done, any spill, or the tin knocked over."""
    return {"open": 1.0, "half": 0.5, "cracked": 0.25}.get(outcome(), 0.0)
