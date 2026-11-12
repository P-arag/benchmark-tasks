"""Geometry of the receipt-printer scene, shared by build_assets.py, task.py and demo.py. Pure Python.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres. Heights in this file are
above the table top (TOP) unless they say "world".

A desktop thermal receipt printer (clamshell, drop-in paper) stands on the table, its front (paper exit and tear
bar) facing the arms. Its lid is shut and latched over an EMPTY paper bucket. A new 80 mm roll lies on the table to
the right, on its side, wound the wrong way round for a straight drop-in: its free end leaves the TOP of the roll
toward the BACK (+x). The sticker in the bucket shows the paper leaving the top of the roll toward the front.

# CHECK (run): every number marked "(sim)" was chosen on paper; confirm it in Isaac Sim before trusting it.
"""

from __future__ import annotations

import math

TOP = 0.7475                                    # yam_arena.task_file.TABLE_TOP["station"]

# -- printer (its origin: the centre of its base; it faces -x, the arms) --------------------------------------------
PRINTER_X, PRINTER_Y = 0.36, 0.04
D, W, H = 0.200, 0.146, 0.148                   # depth (x), width (y), height (z): a common 80 mm desktop model
WALL = 0.006
FRONT_X = -D / 2                                # local x of the front face
HEAD_TOP = H - 0.020                            # the print head / front wall's top edge (local z)
SLOT = 0.003                                    # gap between platen and head when shut, where the paper exits
TEAR_X = FRONT_X                                # the tear bar runs along the front edge

BUCKET_X, BUCKET_Z = 0.010, 0.085               # centre of the paper bucket's half-round cradle (local)
BUCKET_R = 0.0435                               # takes rolls up to 83 mm across
BUCKET_W = 0.083                                # inner width between its side walls

# -- lid: hinged along the back top edge, rotating about +y (positive = front edge up) ------------------------------
HINGE_X, HINGE_Z = D / 2 - 0.005, H
LID_L, LID_W, LID_T = D - 0.005, W - 0.030, 0.004   # leaves a 15 mm deck strip on each side for the button
LID_MASS = 0.15
LID_MAX = math.radians(105)                     # past vertical: the open lid rests back on its stop
POP_ANGLE = math.radians(20)                    # the release spring lifts the lid this far (target), then no further
POP_K = 1.0                                     # N m/rad (sim): enough to lift the 150 g lid ~11 degrees
LATCH_BELOW = math.radians(1.2)                 # pushed shut below this, the latch catches (and holds at 0)
REARM_ABOVE = math.radians(8)                   # the latch only re-catches after the lid has been opened this far
PLATEN_R = 0.008                                # the platen roller under the lid's front edge
PLATEN_LOCAL = (-(LID_L - 0.012), 0.0, -(H - HEAD_TOP) + PLATEN_R + SLOT)   # in the lid frame (origin at hinge)

# -- release button: on the right-hand deck strip near the front, pressed straight down ------------------------------
BUTTON_X, BUTTON_Y = FRONT_X + 0.030, -(W / 2 - 0.0075)
BUTTON_R, BUTTON_H = 0.006, 0.006
BUTTON_TRAVEL = 0.004
BUTTON_PRESSED = 0.003
BUTTON_K = 600.0                                # N/m: ~2.4 N at the bottom, like a real release button

# -- the roll: 80 mm wide, 80 mm across, on a 12 mm core; its free end is a strip of hinged links ------------------
ROLL_R, ROLL_W, CORE_R = 0.040, 0.080, 0.006
ROLL_MASS = 0.30
TAIL_N, TAIL_LINK = 12, 0.013                   # 15.6 cm of free paper
TAIL_T, TAIL_W = 0.0015, 0.078                  # collider thickness (real paper is 0.06 mm: see fidelity)
TAIL_LINK_MASS = 0.002
TAIL_BEND_K, TAIL_BEND_D = 0.0008, 0.0002       # per hinge, N m/rad (sim): paper is limp but not a chain
TAIL_LIMIT = math.radians(120)

# start: lying on its side to the right of the printer, axis along y, free end leaving the top toward the back
ROLL_START = (0.24, -0.20, ROLL_R)               # its centre (local to the table)

# -- scoring --------------------------------------------------------------------------------------------------------
IN_BUCKET_TOL = 0.015                           # roll centre within 1.5 cm of the cradle's centre (x, z)
AXIS_TOL = math.radians(15)                     # roll axis within 15 degrees of the printer's width (y)
OUT_PAST_TEAR = 0.020                           # the free end at least 2 cm out past the tear bar
CORRECT_SENSE = -1                              # sign of ((root - centre) x (tail direction)) . y, see task.py
STILL_S = 1.0
