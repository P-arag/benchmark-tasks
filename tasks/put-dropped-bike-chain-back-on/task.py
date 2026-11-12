"""On the station table: a child's single-speed bicycle drivetrain at real size in a bench stand, the bike's length
running left-right with its drive side toward the arms. The chain has come off the 28-tooth chainring to the inside
and hangs slack round the bottom-bracket shell; it is still on the 16-tooth rear sprocket. The instruction: put the
chain back on. The usual way is to lay the chain over a few teeth of the chainring and turn the crank so the ring
draws the rest on; then the crank should turn a full revolution with the chain staying on.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "The chain has come off. Put it back on." --scorer task_score ...
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
INSTRUCTION = "The chain has come off. Put it back on."
TOP = g.TOP

RING_C = np.array([g.RING_X, g.RING_Y, TOP + g.AXLE_Z])
COG_C = np.array([g.RING_X, g.COG_Y, TOP + g.AXLE_Z])

# -- objects --------------------------------------------------------------------------------------------------------
DRIVETRAIN = UsdBody("drivetrain", usd="assets/drivetrain.usda", pos=(g.RING_X, g.RING_Y, TOP),
                     size=(0.12, g.CHAINSTAY + 0.15, 0.25), articulation=True,
                     joint_pos=(("crank_axle", g.CRANK_START), ("pedal_spindle", -g.CRANK_START), ("cog_axle", 0.0)),
                     drives=(("crank_axle", 0.0, g.CRANK_DAMPING), ("pedal_spindle", 0.0, 0.002),
                             ("cog_axle", 0.0, g.COG_DAMPING)),
                     friction=(0.4, 0.3))
_CHAIN_POS, _CHAIN_QUAT = g.chain_pose_world()
CHAIN = UsdBody("chain", usd="assets/chain.usda", pos=_CHAIN_POS, rot_xyzw=_CHAIN_QUAT,
                size=(0.02, g.CHAINSTAY + 0.15, 0.15), articulation=True,
                drives=((r"side_\d\d", g.SIDE_K, g.SIDE_D),), joint_damping=0.0005,
                friction=(0.3, 0.25), solver_iters=48)
# CHECK (run): scenekit gives every undriven joint (the bend_kk joints) damping joint_damping and no stiffness:
# right for a chain. Watch the first 2 s after reset: the chain must drop from its built loop onto the BB shell
# and hang still, still meshed on the sprocket. Screenshot that state for setup.png and describe it in task.json.
OBJECTS = [DRIVETRAIN, CHAIN]

# -- monitor ---------------------------------------------------------------------------------------------------------
_STATE: dict = {}
_STILL = tk.Still()


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(on_ring=0, on_cog=0, crank=g.CRANK_START, turned=0.0, best_turn=0.0, chain_still=0.0)
    _STILL.reset()


@before_reset
def _restart(environment) -> None:
    write_joint_positions(environment, "drivetrain", {"crank_axle": g.CRANK_START, "pedal_spindle": -g.CRANK_START,
                                                      "cog_axle": 0.0})
    # CHECK (run): the chain's own joints return to 0 (the built loop) because Isaac Lab resets articulations to
    # their default joint positions; confirm it, otherwise write every bend/side joint to 0 here.


_reset_state()


def _rollers(environment) -> np.ndarray:
    """World positions of every roller (each link's front pin)."""
    pts = []
    for name, (pos, quat) in tk.body_poses(environment, "chain").items():
        if name.startswith("link_"):
            pts.append(np.asarray(pos) + tk.rotation(quat)[:, 0] * (g.PITCH / 2))
    return np.array(pts)


def _seated(rollers: np.ndarray, centre: np.ndarray, r_pitch: float) -> int:
    d = rollers - centre
    radial = np.hypot(d[:, 1], d[:, 2])
    ok = (np.abs(radial - r_pitch) < g.SEAT_RADIAL) & (np.abs(d[:, 0]) < g.SEAT_LATERAL)
    return int(ok.sum())


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    st = object_states(environment, ["drivetrain", "chain"])
    rollers = _rollers(environment)
    _STATE["on_ring"] = on_ring = _seated(rollers, RING_C, g.R_RING)
    _STATE["on_cog"] = _seated(rollers, COG_C, g.R_COG)
    crank = float(st["drivetrain"].joints["crank_axle"])
    step = crank - _STATE["crank"]
    _STATE["crank"] = crank
    # count crank rotation (either direction) only while the chain stays on the ring; it starts over if it falls off
    if on_ring >= g.ENGAGED_MIN:
        _STATE["turned"] += abs(step)
    else:
        _STATE["turned"] = 0.0
    _STATE["best_turn"] = max(_STATE["best_turn"], _STATE["turned"])
    c = st["chain"]
    _STATE["chain_still"] = _STILL.update("chain", c.pos, c.quat_xyzw, c.lin_vel, c.ang_vel)


# -- scoring ---------------------------------------------------------------------------------------------------------
def outcome(objects=None) -> str:
    """'done' (back on: >= 10 rollers seated on the chainring now, and the crank has turned a full revolution while
    it stayed on), 'on_not_turned' (back on now but not proven by a full turn), 'started' (4-9 rollers seated), or
    'other' (including a chain knocked off the rear sprocket)."""
    if _STATE["on_cog"] < g.ON_COG_MIN:
        return "other"
    if _STATE["on_ring"] >= g.ENGAGED_MIN:
        if _STATE["turned"] >= g.FULL_TURN and _STATE["chain_still"] >= g.STILL_S:
            return "done"
        return "on_not_turned"
    if _STATE["on_ring"] >= g.STARTED_MIN:
        return "started"
    return "other"


def success(objects) -> bool:
    return outcome() == "done"


def score(objects) -> float:
    """1.0 done; 0.5 the chain is on the chainring but the crank hasn't turned a full revolution with it on (it may
    only be balanced on the teeth); 0.25 the chain is on 4-9 teeth; 0 anything else."""
    return {"done": 1.0, "on_not_turned": 0.5, "started": 0.25}.get(outcome(), 0.0)
