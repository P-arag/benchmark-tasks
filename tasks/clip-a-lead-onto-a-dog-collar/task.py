"""On the station table: a seated plush-dog form facing the arms, wearing a red nylon collar with a steel D-ring that
hangs free at the front of its neck; a 45 cm blue traffic lead lies on the table to the right, its steel bolt snap at
the near end. Instruction: "Clip the lead onto the dog's collar." The bolt snap only opens while its trigger is held
back, so one hand holds the snap while the other works the trigger, and the D-ring swings away if pushed.

Clipped is decided topologically: the snap's closed hook (hook + shut bolt) and the D-ring's wire must be linked
loops (Gauss linking number ~ +-1), which no amount of lying the snap next to the ring can fake.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Clip the lead onto the dog's collar." --scorer task_score ...
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
INSTRUCTION = "Clip the lead onto the dog's collar."
TOP = g.TOP


def _q_mul(a, b):
    """Quaternion product, (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


_ON_SIDE = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))           # +90 deg about x: hook plane flat
_YAW = (0.0, 0.0, math.sin(g.SNAP_START_YAW / 2), math.cos(g.SNAP_START_YAW / 2))
SNAP_QUAT = _q_mul(_YAW, _ON_SIDE)

DOG = UsdBody("dog_collar", usd="assets/dog_collar.usda", pos=(g.DOG_X, g.DOG_Y, TOP), size=(0.25, 0.16, 0.26),
              articulation=True, joint_pos=(("dring_hinge", 0.0),), drives=(("dring_hinge", 0.0, g.D_DAMPING),),
              friction=(0.8, 0.7))
LEAD = UsdBody("lead", usd="assets/lead.usda", pos=(g.SNAP_START[0], g.SNAP_START[1], TOP + g.SNAP_START[2]),
               rot_xyzw=SNAP_QUAT, size=(0.6, 0.03, 0.03), articulation=True,
               joint_pos=(("bolt_slide", 0.0), *((f"lead_{k:02d}", 0.0) for k in range(g.LEAD_LINKS))),
               drives=(("bolt_slide", g.BOLT_K, 0.5), (r"lead_\d\d", g.LEAD_K, g.LEAD_D)),
               friction=(0.6, 0.5), solver_iters=32, torsional_patch=0.004)
# Start pose: the snap lies on its side (hook plane flat on the table), hook pointing at the arms (-x), the lead
# running away from them (+x). Snap frame -> world: x -> -x, y -> +z, z -> +y.
# CHECK (run): the barrel (4.2 mm radius) should rest on the table at SNAP_START z; confirm in the first frame that
# nothing drops or bounces, and that the lead lies flat.
OBJECTS = [DOG, LEAD]

_STATE: dict = {}
_STILL = tk.Still()
_HOOK_SHUT = g.hook_loop(0.0, close_gap=False)
_D = g.d_loop()


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(bolt_open=0.0, max_open=0.0, linked=0.0, inside=0.0, released=False, snap_still=0.0, why=[])
    _STILL.reset()


@before_reset
def _restart(environment) -> None:
    write_joint_positions(environment, "dog_collar", {"dring_hinge": 0.0})
    write_joint_positions(environment, "lead", {"bolt_slide": 0.0})


_reset_state()


def _to_world(points: np.ndarray, pos, quat_xyzw) -> np.ndarray:
    return points @ tk.rotation(quat_xyzw).T + np.asarray(pos)


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    st = object_states(environment, ["lead"])
    snap = st["lead"]
    opened = _STATE["bolt_open"] = -float(snap.joints["bolt_slide"])
    _STATE["max_open"] = max(_STATE["max_open"], opened)
    dring_pos, dring_quat = tk.body_poses(environment, "dog_collar")["dring"]
    d_world = _to_world(_D, dring_pos, dring_quat)
    shut = opened <= g.CLOSED_TOL
    loop = _HOOK_SHUT if shut else g.hook_loop(opened, close_gap=True)
    lk = g.linking_number(_to_world(loop, snap.pos, snap.quat_xyzw), d_world)
    _STATE["linked"] = abs(lk) if shut else 0.0          # captured: a real closed loop round the wire
    _STATE["inside"] = abs(lk)                           # the wire is inside the hook (gate open or shut)
    tips = tk.fingertips(environment)
    _STATE["released"] = bool(tips) and min(np.linalg.norm(np.asarray(p) - np.asarray(snap.pos)) for p in tips) \
        > g.RELEASED_DIST
    _STATE["snap_still"] = _STILL.update("snap", snap.pos, snap.quat_xyzw, snap.lin_vel, snap.ang_vel)


def outcome(objects=None) -> str:
    """'clipped' (linked with the bolt shut, snap let go and hanging still), 'held' (linked and shut but still in
    the hand), 'gate_open' (the D-ring's wire inside the hook but the bolt not shut), 'opened' (the trigger was
    worked: bolt pulled back >= 6 mm at some point), or 'other'."""
    if _STATE["linked"] > 0.5:
        if _STATE["released"] and _STATE["snap_still"] >= g.STILL_S:
            return "clipped"
        return "held"
    if _STATE["inside"] > 0.5:
        return "gate_open"
    if _STATE["max_open"] >= g.OPENED:
        return "opened"
    return "other"


def success(objects) -> bool:
    return outcome() == "clipped"


def score(objects) -> float:
    """1.0 clipped and let go; 0.5 clipped (bolt shut round the ring) but the robot is still holding the snap; 0.25
    the ring inside the hook with the bolt not shut (it would slip off at the first tug); 0 anything else (working the
    trigger alone earns nothing)."""
    return {"clipped": 1.0, "held": 0.5, "gate_open": 0.25}.get(outcome(), 0.0)
