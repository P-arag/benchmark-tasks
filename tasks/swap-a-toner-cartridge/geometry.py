"""Geometry of the toner-swap scene, shared by build_assets.py, task.py and demo.py. Pure Python.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights above the table
top (TOP) unless they say "world".

A compact mono laser printer stands on the table, its front toward the arms. Behind its bottom-hinged front door
(shut, held by a magnet) the used toner cartridge sits on two side rails, held home by a spring detent: a rib under
the cartridge's front edge has snapped past a sprung bump in the floor. A new cartridge, identical but with its
orange protective cover still clipped over the top of its front edge, lies on the table to the right; an open
return box (for recycling the old one) stands to the left. The cover stands 12 mm proud of the cartridge's top, and
the cavity has only 6 mm of headroom: with the cover on, the cartridge can't go home.

# CHECK (run): numbers marked (sim) are first guesses.
"""

from __future__ import annotations

import math

TOP = 0.7475

# -- printer (origin: centre of its base; front face at local x = -D/2) ----------------------------------------------
PRINTER_X, PRINTER_Y = 0.43, 0.0
D, W, H = 0.300, 0.360, 0.190
WALL = 0.008
CAV_Y = 0.155                                   # cavity half-width (cartridge is 0.150 half-length)
FLOOR_Z = 0.040                                 # cavity floor
RAIL_H, RAIL_W = 0.008, 0.012                   # rails along x at the cavity's two sides, under the cartridge ends
CAV_TOP = FLOOR_Z + RAIL_H + 0.090 + 0.006      # 6 mm of headroom over the cartridge
CAV_DEPTH = 0.130                               # from the front face to the cavity's back wall

# -- cartridge (origin: its centre). 30 x 10 x 9 cm, 0.8 kg (a full one; the used one weighs 0.6 kg) --------------
C_L, C_D, C_H = 0.300, 0.100, 0.090             # along y, x, z
NEW_MASS, OLD_MASS = 0.80, 0.60
SEAT_X = -D / 2 + 0.015 + C_D / 2               # local x of a seated cartridge's centre (front 15 mm inside)
SEAT_Z = FLOOR_Z + RAIL_H + C_H / 2
RIB = (0.005, 0.060, 0.005)                     # rib under the front edge (x, y, z), centred in y
RIB_X = -C_D / 2 + 0.012                        # in the cartridge's frame
HANDLE = (0.015, 0.080, 0.012)                  # the handle bar on the front face, 20 mm proud, 15 mm gap behind
HANDLE_OUT = 0.020

# -- detent: a sprung bump in the cavity floor, just in front of the seated rib ------------------------------------
BUMP_R, BUMP_LEN = 0.004, 0.040
BUMP_X = SEAT_X + RIB_X - RIB[0] / 2 - BUMP_R - 0.0005
BUMP_TOP = FLOOR_Z + RAIL_H + 0.002             # the bump stands 2 mm above the rib's lowest point at rest (sim)
BUMP_TRAVEL = 0.008
BUMP_K = 900.0                                  # N/m (sim): about 8-10 N to push the cartridge past it

# -- protective cover: orange, clipped over the new cartridge's top front edge ---------------------------------------
COVER_L = 0.280
COVER_TOP = 0.012                               # how far it stands above the cartridge's top
COVER_MASS = 0.03

# -- front door: bottom-hinged, opens toward the arms (negative angle about +y), magnet catch -----------------------
DOOR_HINGE_Z = 0.030
DOOR_H = CAV_TOP + 0.010 - DOOR_HINGE_Z
DOOR_T = 0.006
DOOR_OPEN = -math.radians(90)
MAGNET_ZONE = math.radians(5)                   # within 5 degrees of shut, the magnet pulls it shut (sim)
MAGNET_K = 3.0                                  # N m/rad: ~2 N at the top edge to pull it open

# -- table layout (world x, y; the new cartridge lies in its install orientation) ----------------------------------
NEW_XY = (0.24, -0.27)
BOX_XY = (0.22, 0.27)                           # the return box's centre
BOX_IN = (0.130, 0.340, 0.100)                  # inside: x, y, depth
BOX_WALL = 0.004

# -- scoring --------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.004                                # centre within 4 mm of home, past the detent
COVER_ON_TOL = 0.015                            # the cover is "on" if within 1.5 cm of its clipped pose
DOOR_SHUT = math.radians(2)
STILL_S = 1.0
