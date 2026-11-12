"""Geometry of the dropped-chain scene, shared by build_assets.py, task.py and demo.py. Pure Python (math only).

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights above the table
top (TOP) unless they say "world".

A child's single-speed bicycle drivetrain, at real size, is clamped in a bench stand with the bike's length running
left-right across the table and its drive side facing the arms: a 28-tooth chainring with its crank and pedal at
the left, a 16-tooth rear sprocket 30 cm to the right, joined by a 1/2 x 1/8 inch chain of 70 links. The chain has
come off the chainring to the inside and hangs slack round the bottom-bracket shell; it is still on the sprocket.

The chain's plane is the yz plane (normal +x). Each link's local frame: X along the chain toward the next link,
Z = world +x (the bend axis), Y = Z x X (the side-flex axis).

# CHECK (run): numbers marked (sim) are first guesses.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475

PITCH = 0.0127                                   # 1/2 inch
RING_T, COG_T = 28, 16
R_RING = PITCH / (2 * math.sin(math.pi / RING_T))   # 56.7 mm pitch radius
R_COG = PITCH / (2 * math.sin(math.pi / COG_T))     # 32.6 mm
TOOTH_H, TOOTH_W, TOOTH_THICK = 0.0055, 0.0040, 0.0030   # tooth above the pitch circle, along the pitch circle, along x
ROLLER_R, INNER_W = 0.0039, 0.0033               # 7.75 mm rollers, 1/8 inch between the inner plates
PLATE_H, PLATE_T = 0.0072, 0.0010                # side plates: height and thickness
LINK_MASS = 0.0050                               # a 70-link 1/8 inch chain weighs about 350 g

# -- the stand: positions of the two axles (world x, y; z above the table) --------------------------------------------
RING_X, RING_Y, AXLE_Z = 0.36, 0.17, 0.13        # bottom-bracket axle
CHAINSTAY = 0.300                                # axle to axle (horizontal dropouts, set for ~1.5 cm of chain slack)
COG_Y = RING_Y - CHAINSTAY
CHAIN_X = RING_X                                 # chainline: the chainring's and sprocket's mid-plane
DROP_X = RING_X + 0.008                          # the dropped chain hangs 8 mm inboard of the ring
BB_SHELL_R, BB_SHELL_LEN = 0.0195, 0.068         # the frame's bottom-bracket shell, round the axle, inboard
CRANK_LEN = 0.095                                # kids' crank arm
PEDAL_X = RING_X - 0.045                         # the pedal's centre, toward the arms
PEDAL_SIZE = (0.070, 0.065, 0.022)               # resin kids' pedal: along its spindle (x), width (y), thickness
CRANK_START = math.radians(30)                   # crank angle at start: 0 = arm straight down, + = about world +x
                                                 # (the pedal swings toward +y, the bike's front)
CRANK_DAMPING = 0.02                             # N m s/rad (sim): bottom-bracket bearings
COG_DAMPING = 0.01

N_LINKS = 70                                     # even; 69.5 needed tight on the ring, so ~7 mm of slack
SIDE_LIMIT = math.radians(4)                     # a chain's lateral flex per joint
SIDE_K, SIDE_D = 0.05, 0.001                     # N m/rad (sim)

# -- scoring --------------------------------------------------------------------------------------------------------
SEAT_RADIAL = 0.003                              # a roller within 3 mm of the pitch circle...
SEAT_LATERAL = 0.003                             # ...and within 3 mm of the ring's plane counts as seated
ENGAGED_MIN = 10                                 # >= 10 rollers on the ring (about 130 degrees of wrap): back on
STARTED_MIN = 4                                  # >= 4 rollers on the ring: started
ON_COG_MIN = 4                                   # still on the rear sprocket
FULL_TURN = 2 * math.pi
STILL_S = 1.0


# -- the dropped chain's starting path ------------------------------------------------------------------------------
def belt_length(r1: float, r2: float, c: float) -> float:
    """Length of an open belt round two circles of radii r1, r2 whose centres are c apart."""
    d = r1 - r2
    return 2 * math.sqrt(c * c - d * d) + math.pi * (r1 + r2) + 2 * d * math.asin(d / c)


def virtual_front_radius() -> float:
    """Radius of the circle (at the axle, in the drop plane) that a tight loop of N_LINKS round the sprocket would
    need. The chain is built round it and, released at reset, falls slack onto the BB shell. Bisection."""
    target = N_LINKS * PITCH
    lo, hi = 0.001, 0.2
    for _ in range(80):
        mid = (lo + hi) / 2
        if belt_length(mid, R_COG, CHAINSTAY) < target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def loop_points(n: int, r_front: float) -> list[tuple[float, float]]:
    """``n`` points (y, z relative to the BB axle) spaced one pitch apart along the tight loop round the front circle
    (radius r_front at the axle) and the sprocket (at -CHAINSTAY), clockwise seen from +x, starting at the top of
    the front circle."""
    c, r1, r2 = CHAINSTAY, r_front, R_COG
    phi = math.asin((r1 - r2) / c)
    # tangent points (top run and bottom run), front circle centred at (0,0), rear at (-c, 0)
    a_top, a_bot = math.pi / 2 - phi, -math.pi / 2 + phi        # angles on both circles where the runs touch
    segs = []
    # front arc: from the top tangent going clockwise (decreasing angle) round the front to the bottom tangent
    segs.append(("arc", (0.0, 0.0), r1, a_top, a_bot - 0.0))
    p_bf = (r1 * math.cos(a_bot), r1 * math.sin(a_bot))
    p_br = (-c + r2 * math.cos(a_bot), r2 * math.sin(a_bot))
    segs.append(("line", p_bf, p_br))
    # rear arc: from the bottom tangent clockwise round the back to the top tangent
    segs.append(("arc", (-c, 0.0), r2, a_bot, a_top - 2 * math.pi))
    p_tr = (-c + r2 * math.cos(a_top), r2 * math.sin(a_top))
    p_tf = (r1 * math.cos(a_top), r1 * math.sin(a_top))
    segs.append(("line", p_tr, p_tf))

    def seg_len(s):
        if s[0] == "line":
            return math.dist(s[1], s[2])
        return abs(s[4] - s[3]) * s[2]

    def point_at(s, t):
        if s[0] == "line":
            (y0, z0), (y1, z1) = s[1], s[2]
            f = t / seg_len(s)
            return (y0 + f * (y1 - y0), z0 + f * (z1 - z0))
        (cy, cz), r, a0, a1 = s[1], s[2], s[3], s[4]
        a = a0 + (a1 - a0) * t / seg_len(s)
        return (cy + r * math.cos(a), cz + r * math.sin(a))

    lens = [seg_len(s) for s in segs]
    total = sum(lens)
    pts = []
    for k in range(n):
        t = k * total / n                                # spread evenly: chords on the arcs are a hair short
        for s, ln in zip(segs, lens):
            if t <= ln:
                pts.append(point_at(s, t))
                break
            t -= ln
    return pts


def link_frames() -> list[tuple[float, float, float]]:
    """Each link's centre (y, z relative to the BB axle) and direction angle in the chain plane, as built."""
    pts = loop_points(N_LINKS, virtual_front_radius())
    out = []
    for i in range(N_LINKS):
        (y0, z0), (y1, z1) = pts[i], pts[(i + 1) % N_LINKS]
        out.append(((y0 + y1) / 2, (z0 + z1) / 2, math.atan2(z1 - z0, y1 - y0)))
    return out


def link_rotation(ang: float) -> np.ndarray:
    """A link's frame in the world: X along the chain (0, cos, sin), Z = world +x (bend axis), Y = Z x X."""
    X = np.array([0.0, math.cos(ang), math.sin(ang)])
    Z = np.array([1.0, 0.0, 0.0])
    return np.column_stack([X, np.cross(Z, X), Z])


def quat_wxyz(R: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (w, x, y, z)


def chain_pose_world():
    """Link 0's world pose: the chain asset's origin (position, quaternion xyzw)."""
    y0, z0, a0 = link_frames()[0]
    w, x, y, z = quat_wxyz(link_rotation(a0))
    return (DROP_X, RING_Y + y0, TOP + AXLE_Z + z0), (x, y, z, w)
