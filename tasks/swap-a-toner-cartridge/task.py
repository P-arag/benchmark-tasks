"""On the station table: a compact mono laser printer, front toward the arms, its front door shut over a used toner
cartridge; a new cartridge (orange protective cover still clipped over its top front edge) lying on two foam
packing blocks to the right; an open cardboard return box to the left. Instruction: "Replace the toner cartridge
with the new one." Unstated but needed: take the cover off the new cartridge (with it on, the cartridge can't go
home), put the old one in the return box, and shut the door.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Replace the toner cartridge with the new one." --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, on_reset, on_step, write_joint_positions  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

import geometry as g  # noqa: E402
import taskkit as tk  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Replace the toner cartridge with the new one."
TOP = g.TOP

P = np.array([g.PRINTER_X, g.PRINTER_Y, TOP])
SEAT_W = P + [g.SEAT_X, 0, g.SEAT_Z]
NEW_W = np.array([g.NEW_XY[0], g.NEW_XY[1], TOP + 0.010 + g.C_H / 2])

PRINTER = UsdBody("laser_printer", usd="assets/laser_printer.usda", pos=tuple(P), size=(g.D, g.W, g.H),
                  articulation=True, joint_pos=(("door_hinge", 0.0), ("detent", 0.0)),
                  drives=(("door_hinge", g.MAGNET_K, 0.05), ("detent", g.BUMP_K, 1.0)), friction=(0.4, 0.3))
OLD = UsdBody("cartridge_old", usd="assets/cartridge_old.usda", pos=tuple(SEAT_W), size=(g.C_D, g.C_L, g.C_H),
              mass=g.OLD_MASS, friction=(0.4, 0.3))
NEW = UsdBody("cartridge_new", usd="assets/cartridge_new.usda", pos=tuple(NEW_W), size=(g.C_D, g.C_L, g.C_H),
              mass=g.NEW_MASS, friction=(0.4, 0.3))
COVER_W = NEW_W + [-g.C_D / 2, 0, g.C_H / 2]
COVER = UsdBody("cover", usd="assets/cover.usda", pos=tuple(COVER_W), size=(0.03, g.COVER_L, g.COVER_TOP),
                mass=g.COVER_MASS, friction=(0.9, 0.8))
# foam packing blocks under the new cartridge's ends (fixed), and the return box (fixed: taped down)
FOAM = [Box(f"foam_{s}", pos=(g.NEW_XY[0], g.NEW_XY[1] + d, TOP + 0.005), size=(0.10, 0.04, 0.010),
            color=(0.92, 0.92, 0.95)) for s, d in (("l", 0.12), ("r", -0.12))]
bx, by = g.BOX_XY
ix, iy, dz = g.BOX_IN
t = g.BOX_WALL
CARDBOARD = (0.62, 0.47, 0.30)
RETURN_BOX = [
    Box("box_floor", pos=(bx, by, TOP + t / 2), size=(ix + 2 * t, iy + 2 * t, t), color=CARDBOARD),
    # the front flap is folded down to 4 cm, so a hand holding a cartridge by its front handle can lower it in
    Box("box_front", pos=(bx - ix / 2 - t / 2, by, TOP + 0.02), size=(t, iy + 2 * t, 0.04), color=CARDBOARD),
    Box("box_back", pos=(bx + ix / 2 + t / 2, by, TOP + dz / 2), size=(t, iy + 2 * t, dz), color=CARDBOARD),
    Box("box_left", pos=(bx, by + iy / 2 + t / 2, TOP + dz / 2), size=(ix, t, dz), color=CARDBOARD),
    Box("box_right", pos=(bx, by - iy / 2 - t / 2, TOP + dz / 2), size=(ix, t, dz), color=CARDBOARD),
]
OBJECTS = [PRINTER, OLD, NEW, COVER, *FOAM, *RETURN_BOX]
# CHECK (run): the old cartridge must settle on its rails, rib behind the bump, without being shot out at reset.
# If PhysX pushes it out, start the bump at its lowest point (detent = -BUMP_TRAVEL) for the first reset step.

_STATE: dict = {}
_STILL = tk.Still()


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(door=0.0, dropped=False, still={}, why=[])
    _STILL.reset()


@before_reset
def _restart(environment) -> None:
    write_joint_positions(environment, "laser_printer", {"door_hinge": 0.0, "detent": 0.0})


_reset_state()


@on_step
def _monitor(environment) -> None:
    from scenekit import set_joint_targets
    from yam_arena.scene import object_states

    st = object_states(environment, ["laser_printer", "cartridge_old", "cartridge_new", "cover"])
    door = _STATE["door"] = float(st["laser_printer"].joints["door_hinge"])
    # the magnet only pulls within a few degrees of shut; elsewhere the door is free (target follows it)
    set_joint_targets(environment, "laser_printer", {"door_hinge": 0.0 if abs(door) < g.MAGNET_ZONE else door})
    for name in ("cartridge_old", "cartridge_new", "cover"):
        s = st[name]
        _STATE["still"][name] = _STILL.update(name, s.pos, s.quat_xyzw, s.lin_vel, s.ang_vel)
        if s.pos[2] < TOP - 0.05 and not _STATE["dropped"]:
            _STATE["dropped"] = True
            _STATE["why"].append(f"{name} fell off the table")
    _STATE["poses"] = {n: (np.asarray(st[n].pos), np.asarray(st[n].quat_xyzw)) for n in st if n != "laser_printer"}


def _seated(name: str) -> bool:
    pos, quat = _STATE["poses"][name]
    local = pos - P
    level = tk.tilt(quat) < math.radians(3)
    return bool(level and local[0] >= g.SEAT_X - g.SEAT_TOL and abs(local[1]) < 0.01
                and abs(local[2] - g.SEAT_Z) < 0.005)


def _cover_on() -> bool:
    (cp, cq), (vp, _) = _STATE["poses"]["cartridge_new"], _STATE["poses"]["cover"]
    edge = cp + tk.rotation(cq) @ np.array([-g.C_D / 2, 0, g.C_H / 2])
    return bool(np.linalg.norm(vp - edge) < g.COVER_ON_TOL)


def _in_box(name: str) -> bool:
    pos, _ = _STATE["poses"][name]
    bx, by = g.BOX_XY
    return bool(abs(pos[0] - bx) < g.BOX_IN[0] / 2 and abs(pos[1] - by) < g.BOX_IN[1] / 2
                and pos[2] < TOP + g.BOX_IN[2] and _STATE["still"].get(name, 0) >= g.STILL_S)


def outcome(objects=None) -> str:
    """'done', 'old_not_boxed' (new seated, cover off, door shut, old cartridge not in the return box),
    'door_open' (new seated, door not shut), or 'other' (including anything dropped off the table)."""
    if "poses" not in _STATE or _STATE["dropped"]:
        return "other"
    new_home = _seated("cartridge_new") and not _cover_on() and _STATE["still"].get("cartridge_new", 0) >= g.STILL_S
    if not new_home:
        return "other"
    if abs(_STATE["door"]) > g.DOOR_SHUT:
        return "door_open"
    return "done" if _in_box("cartridge_old") else "old_not_boxed"


def success(objects) -> bool:
    return outcome() == "done"


def score(objects) -> float:
    """1.0 new cartridge home with its cover off, old one in the return box, door shut; 0.5 the same but the old
    cartridge not in the box; 0.25 the new cartridge home but the door left open; 0 anything else (the new cartridge
    not home, its cover still on, the old one put back, or anything dropped off the table)."""
    return {"done": 1.0, "old_not_boxed": 0.5, "door_open": 0.25}.get(outcome(), 0.0)
