"""Geometry of the ring-pull can scene, shared by build_assets.py, task.py and demo.py. Pure Python.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights above the table
top (TOP) unless they say "world".

A full 400 g tin of soup with a full-aperture easy-open (ring-pull) end stands free on the table. Its ring faces the
arms (-x); the rivet is near the lid's centre and the lid's far edge (+x) is where the peel ends. Opening it takes
two stages: lift the ring about the rivet until the nose cracks the score (a pop), then pull the ring up and back so
the lid tears round its score and folds back off the opening.

# CHECK (run): numbers marked (sim) are first guesses.
"""

from __future__ import annotations

import math

TOP = 0.7475

CAN_X, CAN_Y = 0.34, 0.0
CAN_R, CAN_H = 0.0365, 0.110                    # 73 mm across, 110 mm tall (a 400 g soup tin)
CAN_MASS = 0.45                                 # with contents
FILL_GAP = 0.010                                # the soup's surface is 1 cm below the rim
RIM_H = 0.003                                   # the double seam stands 3 mm above the lid

LID_R = CAN_R - 0.004
LID_T = 0.0008                                  # collider 0.8 mm (real: 0.2 mm tinplate)
LID_Z = CAN_H - 0.002                           # the lid sits 2 mm below the seam's top (local, from the base)
HINGE_X = LID_R                                 # the lid tears off last at its far edge: hinge there (local x)

RIVET_X = -0.006                                # rivet, a little toward the ring side of centre
TAB_L, TAB_W, TAB_T = 0.030, 0.019, 0.0010      # ring tab: 30 mm from nose to ring end, 19 mm wide
TAB_NOSE = 0.006                                # nose length past the rivet (toward +x)
RING_HOLE = (0.014, 0.011)                      # the finger hole in the ring (x, y), visual
RECESS_DEPTH = 0.004                            # the finger dimple under the ring end (sim: deepened from ~3 mm)
RING_RAISE = 0.0015                             # the ring end sits this much above the lid

TAB_K = 0.30                                    # N m/rad before the crack: ~15 N at the ring end to crack (sim)
TAB_D = 0.002
CRACK_ANGLE = math.radians(55)                  # lifted this far about the rivet, the nose cracks the score
TAB_MAX = math.radians(110)
PEEL_DAMPING = 2.0                              # N m s/rad (sim): ~25-30 N of pull at a steady tear
PEEL_MAX = math.radians(175)

# -- scoring --------------------------------------------------------------------------------------------------------
SPILL_TILT = math.atan(2 * FILL_GAP / (2 * CAN_R))   # ~15 degrees: soup 1 cm below the rim reaches it
OPEN_ANGLE = math.radians(100)                  # lid folded back past vertical: the opening is clear
HALF_OPEN = math.radians(45)
STILL_S = 1.0
