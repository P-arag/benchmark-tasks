"""Geometry of the stroller-folding scene, shared by build_assets.py, task.py, demo.py and reach_check.py.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights above the table
top (TOP) unless they say "world".

A compact umbrella-type travel stroller stands open on its four wheels on the table, its push handles toward the
arms. Two release buttons, one on each side at the main fold hinge, must be pressed in at the same time to free the
fold; the frame then folds (the front legs swing back against the rear legs) and a storage latch catches it folded.

!!! FEASIBILITY: a real compact stroller's hinge is ~45 cm and its handles ~95 cm above the surface it stands on.
!!! The YAM arms reach about 20 cm above the table with a straight-down grip. Run reach_check.py BEFORE building
!!! anything else. Scaling the stroller down would be flagged "Miniaturization" (a final rejection), so if the
!!! points are out of reach, drop this task or switch to the fallback in README.md.
"""

from __future__ import annotations

import math

TOP = 0.7475

ORIGIN_X, ORIGIN_Y = 0.40, 0.0                  # the point on the table under the main hinge
HINGE_Z = 0.45                                  # main fold hinge height
HALF_W = 0.22                                   # frame half-width (45 cm wide)
TUBE_R = 0.011                                  # 22 mm aluminium tubes
WHEEL_R, WHEEL_W = 0.075, 0.03
FRONT_WHEEL_X = 0.30                            # front wheels (away from the arms), relative to the hinge
REAR_WHEEL_X = -0.22                            # rear wheels (toward the arms)
HANDLE_TOP = (-0.30, 0.95)                      # push-handle tops (x rel. hinge, z above table)
FRAME_MASS_REAR, FRAME_MASS_FRONT = 3.5, 2.5    # ~6 kg stroller
WHEEL_FRICTION = (0.15, 0.10)                   # (sim) stands in for rolling; wheels have no joints

BUTTON_TRAVEL, BUTTON_PRESSED = 0.006, 0.004
BUTTON_K = 2500.0                               # N/m: ~15 N each, pressed together (child-safety two-button release)
BUTTON = (0.012, 0.025, 0.025)

FOLD_MAX = math.atan2(FRONT_WHEEL_X, HINGE_Z - WHEEL_R) + math.atan2(-REAR_WHEEL_X, HINGE_Z - WHEEL_R) - 0.05
FOLD_LATCH = FOLD_MAX - math.radians(4)         # the storage latch catches here
FOLD_DAMPING = 0.5                              # N m s/rad (sim)
UNLOCK_KEEP = math.radians(8)                   # once folded past this, letting go of the buttons doesn't re-lock

# -- scoring --------------------------------------------------------------------------------------------------------
HALF_FOLD = math.radians(20)
TIPPED = math.radians(60)
STAND_TILT = math.radians(30)
STILL_S = 1.0


def key_points_world():
    """The points a robot must reach (world), for reach_check.py."""
    ox, oy = ORIGIN_X, ORIGIN_Y
    return {
        "button_left": (ox, oy + HALF_W + BUTTON[1] / 2, TOP + HINGE_Z),
        "button_right": (ox, oy - HALF_W - BUTTON[1] / 2, TOP + HINGE_Z),
        "handle_left": (ox + HANDLE_TOP[0], oy + HALF_W, TOP + HANDLE_TOP[1]),
        "handle_right": (ox + HANDLE_TOP[0], oy - HALF_W, TOP + HANDLE_TOP[1]),
        "front_crossbar": (ox + FRONT_WHEEL_X * 0.6, oy, TOP + HINGE_Z * 0.4),
    }
