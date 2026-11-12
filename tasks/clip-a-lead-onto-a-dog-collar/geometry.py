"""Geometry of the lead-and-collar scene, shared by build_assets.py, task.py and demo.py. Pure Python (+ numpy).

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights above the table
top (TOP) unless they say "world".

A dog-sized plush form sits on the table facing the arms, wearing a nylon collar with a steel D-ring at the front
that hangs on its own hinge and swings freely. A short (45 cm) nylon traffic lead lies on the table to the right,
its steel bolt snap at the near end. A bolt snap only opens while its trigger is pulled back toward the swivel
eye; let go, the spring drives the bolt shut against the hook's tip.

Snap frame (the snap's file origin): x along the barrel toward the hook, z in the hook's plane "up", y normal to it.
D-ring frame: origin on its hinge bar's centre, bar along y, the D hangs toward -z at rest, in the yz plane.

# CHECK (run): numbers marked (sim) are first guesses.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475

# -- dog form and collar ------------------------------------------------------------------------------------------
DOG_X, DOG_Y = 0.42, 0.04
NECK_R = 0.055                                   # collar circumference ~35 cm: a medium dog
NECK_Z = 0.140                                   # neck centre height above the table (sitting form)
COLLAR_W = 0.020
DRING_HINGE = (DOG_X - NECK_R - 0.004, DOG_Y, NECK_Z - 0.035)   # world x, y; z above the table (front, low)
D_HALF = 0.0125                                  # 25 mm D-ring
D_WIRE_R = 0.00175                               # 3.5 mm wire
D_LEG = 0.006
D_DAMPING = 0.0005

# -- bolt snap ------------------------------------------------------------------------------------------------------
WIRE_R = 0.0022
BARREL_R, BARREL = 0.0042, 0.045                 # barrel from x = -BARREL to 0
EYE_R = 0.009                                    # swivel eye behind the barrel
HOOK_XZ = [(0.000, 0.004), (0.012, 0.010), (0.022, 0.012), (0.030, 0.006), (0.031, -0.004),
           (0.024, -0.011), (0.014, -0.012), (0.009, -0.008)]          # from the barrel's top round to the tip
BOLT_Z = -0.006
BOLT_LEN = 0.012
BOLT_CLOSED_TIP = 0.008                          # the bolt's tip x when shut (against the hook's tip)
BOLT_TRAVEL = 0.010
BOLT_K = 300.0                                   # N/m (sim): ~3 N to pull fully back
TRIGGER = (0.006, 0.005, 0.006)                  # the thumb tab on the bolt, sticking up through the barrel's slot
SNAP_MASS = 0.030

LEAD_LINKS, LEAD_LINK = 10, 0.045                # 45 cm traffic lead
LEAD_W, LEAD_T = 0.020, 0.002
LEAD_LINK_MASS = 0.004
LEAD_K, LEAD_D = 0.004, 0.0005                   # webbing stiffness per hinge (sim)

SNAP_START = (0.24, -0.16, 0.0045)               # lying on its side on the table (hook plane horizontal), lead toward +x
SNAP_START_YAW = math.pi                         # hook pointing toward the arms (-x)

# -- scoring --------------------------------------------------------------------------------------------------------
CLOSED_TOL = 0.0015                              # the bolt within 1.5 mm of shut
OPENED = 0.006                                   # trigger pulled back at least 6 mm at some point
RELEASED_DIST = 0.03                             # no finger within 3 cm of the snap
STILL_S = 1.0


def hook_loop(bolt_open: float, close_gap: bool) -> np.ndarray:
    """Closed polygon (snap frame) round the inside of the hook. With the bolt shut it is the real loop; with it open
    and ``close_gap`` the gap is bridged by a straight segment (for the "ring inside the hook" test)."""
    pts = [(x, 0.0, z) for x, z in HOOK_XZ]
    tip = BOLT_CLOSED_TIP - bolt_open
    if bolt_open > CLOSED_TOL and not close_gap:
        return np.array([])
    pts += [(tip, 0.0, BOLT_Z), (-0.002, 0.0, BOLT_Z)]
    return np.array(pts)


def d_loop(n_arc: int = 16) -> np.ndarray:
    """Closed polygon of the D-ring's wire (D-ring frame)."""
    pts = [(0.0, -D_HALF, 0.0), (0.0, D_HALF, 0.0), (0.0, D_HALF, -D_LEG)]
    for k in range(1, n_arc):
        a = math.pi * k / n_arc
        pts.append((0.0, D_HALF * math.cos(a), -D_LEG - D_HALF * math.sin(a)))
    pts.append((0.0, -D_HALF, -D_LEG))
    return np.array(pts)


def linking_number(a: np.ndarray, b: np.ndarray) -> float:
    """Gauss linking number of two closed polygons (rows are vertices; each closes back to its first). About +-1 if
    the loops are linked (one passes through the other), about 0 if not."""
    if len(a) < 3 or len(b) < 3:
        return 0.0
    total = 0.0
    for i in range(len(a)):
        p0, p1 = a[i], a[(i + 1) % len(a)]
        for j in range(len(b)):
            q0, q1 = b[j], b[(j + 1) % len(b)]
            total += _segment_solid_angle(p0, p1, q0, q1)
    return total / (4 * math.pi)


def _segment_solid_angle(a0, a1, b0, b1) -> float:
    """Exact contribution of two straight segments to the Gauss integral (Klenin & Langowski 2000)."""
    r13, r14, r23, r24 = b0 - a0, b1 - a0, b0 - a1, b1 - a1
    n = [np.cross(r13, r14), np.cross(r14, r24), np.cross(r24, r23), np.cross(r23, r13)]
    norms = [np.linalg.norm(v) for v in n]
    if min(norms) < 1e-12:
        return 0.0
    n = [v / nv for v, nv in zip(n, norms)]
    om = sum(math.asin(max(-1.0, min(1.0, float(np.dot(n[k], n[(k + 1) % 4]))))) for k in range(4))
    sign = np.sign(np.dot(np.cross(b1 - b0, a1 - a0), r13))
    return float(om * sign)
