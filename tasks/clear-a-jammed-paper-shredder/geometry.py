"""Geometry of the jammed-shredder scene, shared by build_assets.py, task.py and demo.py. Pure Python.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights above the table
top (TOP) unless they say "world".

A desktop cross-cut paper shredder (cutting head on a clear wastebasket) stands on the table, its long feed slot
running left-right. A folded wad of paper is jammed in the slot: 2.5 cm of it sticks up out of the slot, the rest
is wedged down onto the cutters. The head's three-position slide switch, at the front right of its top, is at AUTO
(in AUTO a real shredder starts its cutters the moment anything enters the slot). The shredder's plug is in a socket
behind the table, out of reach.

# CHECK (run): numbers marked (sim) are first guesses.
"""

from __future__ import annotations

TOP = 0.7475

SHRED_X, SHRED_Y = 0.36, 0.02
HEAD_D, HEAD_W, HEAD_H = 0.140, 0.300, 0.060     # cutting head: depth (x), width (y), height (z)
BASKET_D, BASKET_W, BASKET_H = 0.130, 0.280, 0.120
TOTAL_H = BASKET_H + HEAD_H                      # 18 cm: a compact desk-side model
SLOT_LEN, SLOT_W = 0.230, 0.005                  # the feed slot, centred in the head's top
CUTTER_R = 0.012
CUTTER_Z = TOTAL_H - 0.032                       # cutter shafts' axes (two, along y, either side of the slot)
CUTTER_GAP = 0.002

# -- the jam: a folded wad, a rigid body --------------------------------------------------------------------------
WAD_W = 0.150                                    # along y
WAD_SHOW = 0.025                                 # sticks up this far above the slot
WAD_SHEET_T = 0.0015                             # the visible part: a sheet (collider thickness; real paper 0.1 mm)
WAD_JAM_T = 0.0045                               # the folded part in the slot: tight in the 5 mm slot
WAD_JAM_DEPTH = TOTAL_H - (CUTTER_Z + CUTTER_R)  # from the slot's top down to the cutters
WAD_MASS = 0.012
JAM_HOLD = 8.0                                   # N (sim): the cutters' grip on the wad, resisting a pull
FREE_ABOVE = 0.008                               # once the wad's bottom is within 8 mm of the slot's top, it's free

# -- the switch: a knob sliding along y in a short track on the head's top, front right, clear of the slot ---------
SWITCH_X = -0.052                                # in front of the slot (the zone ends 2.5 cm + margin from it)
SWITCH_Y = -0.110                                # track centre (local)
POS = {"REV": -0.012, "OFF": 0.0, "AUTO": +0.012}  # knob offset along y from the track centre
KNOB = (0.016, 0.012, 0.012)
DETENT_K = 60.0                                  # N/m (sim): ~0.7 N to move between positions
REV_MOMENTARY = True                             # released in REV, the knob springs back to OFF

# -- safety zone: the slot's mouth --------------------------------------------------------------------------------
ZONE_HALF = (0.025, SLOT_LEN / 2 + 0.005)        # x, y half-extents around the slot
ZONE_Z = (TOTAL_H - 0.02, TOTAL_H + 0.04)        # from 2 cm below the slot's top to 4 cm above it
ZONE_MARGIN = 0.013                              # finger-link origins sit ~13 mm behind the pad tips

# -- scoring ------------------------------------------------------------------------------------------------------
HALF_OUT = WAD_JAM_DEPTH / 2
STARTED = 0.010
STILL_S = 1.0
