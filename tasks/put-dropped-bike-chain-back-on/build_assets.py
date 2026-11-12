"""Write the drivetrain and chain USD assets from geometry.py.

    python build_assets.py          # Isaac Sim's Python (pxr); no simulator needed

- assets/drivetrain.usda: fixed-base articulation. Root ``stand`` (base plate, upright, bottom-bracket shell,
  chainstay tube, dropout plates); ``crank`` on revolute ``crank_axle`` (about x): 28T chainring (teeth are boxes),
  crank arm; ``pedal`` on revolute ``pedal_spindle`` (about x) at the crank's end; ``cog`` on revolute ``cog_axle``:
  16T sprocket.
- assets/chain.usda: floating articulation, 70 links. Link k is joined to link k+1 by revolute ``bend_kk`` (about the
  link's local z = world x) into a small ``hinge_kk`` body, then revolute ``side_kk`` (about local y, +/-4 degrees)
  into link k+1. The loop is closed by a spherical joint ``close`` from link 69 to link 0 that is EXCLUDED from the
  articulation (PhysX articulations must be trees).

The chain is built in the drop plane (8 mm inboard of the ring) along a tight loop round the sprocket and a virtual
circle at the axle; at reset it falls slack onto the BB shell. Its file origin is link 0's frame, so task.py places
it by link 0's world pose (see CHAIN_POSE).

# CHECK (run), most likely to need work:
#   - The closing joint: confirm Isaac Lab accepts an articulation with an excluded loop joint (physics:
#     excludeFromArticulation = true). If it doesn't, drop "close" and anchor link 0 to the sprocket with a fixed
#     joint instead (the chain can't come off the sprocket then, which this task doesn't need anyway).
#   - Stability: 70 links + 69 hinge bodies at 5 g / 0.5 g. If the chain jitters or stretches, raise
#     solver_iters (task.py), raise hinge masses to 1 g, or set physxScene substeps higher.
#   - Rollers vs teeth: the ring's teeth (3 mm) must fit between the inner plates (3.3 mm apart).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Sdf

import geometry as g
from usdkit import (articulation_root, body, box, cylinder, fixed, joint, material, no_collide, quat_axis_angle,
                    stage_for)

OUT = Path(__file__).resolve().parent / "assets"
STEEL = (0.55, 0.56, 0.58)
BLACK = (0.08, 0.08, 0.09)
FRAME = (0.15, 0.35, 0.75)


def teeth(stage, prefix, n_teeth, r_pitch, mat):
    """Teeth between roller seats: seat k at angle 2*pi*k/n; tooth centred half a pitch further on."""
    for k in range(n_teeth):
        a = 2 * math.pi * (k + 0.5) / n_teeth
        rr = r_pitch + g.TOOTH_H / 2 - 0.001
        # the tooth's long side is radial: rotate a box about x by the angle
        box(stage, f"{prefix}_{k:02d}", (g.TOOTH_THICK, g.TOOTH_W, g.TOOTH_H + 0.004),
            (0, rr * math.cos(a), rr * math.sin(a)), mat=mat, rot_wxyz=quat_axis_angle((1, 0, 0), a - math.pi / 2))


def build_drivetrain() -> None:
    stage = stage_for(OUT, "drivetrain")
    r = "/drivetrain"
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.35, metallic=0.9)
    black = material(stage, f"{r}/Looks/black", BLACK, roughness=0.6)
    paint = material(stage, f"{r}/Looks/paint", FRAME, roughness=0.4)
    # the asset's origin: the bottom-bracket axle's centre at table height (task.py places it at RING_X, RING_Y, TOP)
    s = f"{r}/stand"
    body(stage, s, 6.0)
    articulation_root(stage, s, fixed=True)
    box(stage, f"{s}/base", (0.12, 0.45, 0.012), (0.03, -g.CHAINSTAY / 2, 0.006), mat=black)
    box(stage, f"{s}/upright_front", (0.03, 0.03, g.AXLE_Z), (0.05, 0.0, g.AXLE_Z / 2), mat=black)
    box(stage, f"{s}/upright_rear", (0.03, 0.03, g.AXLE_Z), (0.05, -g.CHAINSTAY, g.AXLE_Z / 2), mat=black)
    cylinder(stage, f"{s}/bb_shell", g.BB_SHELL_R, g.BB_SHELL_LEN,
             (g.DROP_X - g.RING_X - 0.004 + g.BB_SHELL_LEN / 2, 0, g.AXLE_Z), axis="X", mat=paint)
    # chainstay: a tube from the shell to the rear dropout, inboard of the chain
    cylinder(stage, f"{s}/chainstay", 0.009, g.CHAINSTAY, (0.030, -g.CHAINSTAY / 2, g.AXLE_Z), axis="Y", mat=paint)
    box(stage, f"{s}/dropout", (0.006, 0.05, 0.03), (0.022, -g.CHAINSTAY, g.AXLE_Z), mat=paint)

    # crank: chainring + arm, on the bottom-bracket axle
    c = f"{r}/crank"
    body(stage, c, 0.45, (0, 0, g.AXLE_Z))
    cylinder(stage, f"{c}/ring_disc", g.R_RING - 0.006, g.TOOTH_THICK, (0, 0, 0), axis="X", mat=steel)
    teeth(stage, f"{c}/tooth", g.RING_T, g.R_RING, steel)
    cylinder(stage, f"{c}/axle", 0.010, 0.09, (0.02, 0, 0), axis="X", mat=steel)
    arm_mid = g.CRANK_LEN / 2
    box(stage, f"{c}/arm", (0.012, 0.022, g.CRANK_LEN + 0.02), (-0.016, 0, -arm_mid), mat=black)
    # the arm is built pointing straight down; the start angle is set by task.py's joint_pos
    joint(stage, f"{r}/joints/crank_axle", "angular", s, c, (0, 0, g.AXLE_Z), axis="X")

    p = f"{r}/pedal"
    body(stage, p, 0.18, (g.PEDAL_X - g.RING_X, 0, g.AXLE_Z - g.CRANK_LEN))
    box(stage, f"{p}/body", g.PEDAL_SIZE, (0, 0, 0), mat=black)
    joint(stage, f"{r}/joints/pedal_spindle", "angular", c, p, (g.PEDAL_X - g.RING_X, 0, -g.CRANK_LEN), axis="X")
    no_collide(stage, p, c)

    k = f"{r}/cog"
    body(stage, k, 0.12, (0, -g.CHAINSTAY, g.AXLE_Z))
    cylinder(stage, f"{k}/disc", g.R_COG - 0.005, g.TOOTH_THICK, (0, 0, 0), axis="X", mat=steel)
    teeth(stage, f"{k}/tooth", g.COG_T, g.R_COG, steel)
    cylinder(stage, f"{k}/hub", 0.018, 0.06, (0.03, 0, 0), axis="X", mat=steel)
    joint(stage, f"{r}/joints/cog_axle", "angular", s, k, (0, -g.CHAINSTAY, g.AXLE_Z), axis="X")
    for part in (c, k):
        no_collide(stage, part, s)
    stage.GetRootLayer().Save()


def build_chain() -> None:
    stage = stage_for(OUT, "chain")
    r = "/chain"
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.4, metallic=0.9)
    links = g.link_frames()
    # file frame = link 0's frame: express every link relative to it
    y0, z0, a0 = links[0]
    R0 = g.link_rotation(a0)
    p0 = np.array([0.0, y0, z0])
    half = g.PITCH / 2
    for i, (y, z, a) in enumerate(links):
        Rw = g.link_rotation(a)
        Rl = R0.T @ Rw
        pl = R0.T @ (np.array([0.0, y, z]) - p0)
        path = f"{r}/link_{i:02d}"
        body(stage, path, g.LINK_MASS, tuple(pl), g.quat_wxyz(Rl))
        # roller at the link's front pin, and two side plates straddling the teeth
        cylinder(stage, f"{path}/roller", g.ROLLER_R, g.INNER_W - 0.0002, (half, 0, 0), mat=steel)
        for s, zz in (("in", (g.INNER_W + g.PLATE_T) / 2), ("out", -(g.INNER_W + g.PLATE_T) / 2)):
            box(stage, f"{path}/plate_{s}", (g.PITCH + 0.004, g.PLATE_H, g.PLATE_T), (0, 0, zz), mat=steel)
        if i == 0:
            articulation_root(stage, path, fixed=False)
    for i in range(g.N_LINKS):
        j = (i + 1) % g.N_LINKS
        a, b = f"{r}/link_{i:02d}", f"{r}/link_{j:02d}"
        if j == 0:
            # loop closure, outside the articulation
            cj = fixed(stage, f"{r}/joints/close", a, b, (half, 0, 0), (-half, 0, 0))
            # CHECK (run): a fixed joint is too stiff here; replace with UsdPhysics.SphericalJoint if PhysX fights it.
            cj.GetPrim().CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)
            continue
        Ra, Rb = g.link_rotation(links[i][2]), g.link_rotation(links[j][2])
        rel = g.quat_wxyz(Ra.T @ Rb)                        # as built, the joint angle is 0
        h = f"{r}/hinge_{i:02d}"
        hinge_pos = np.array(stage.GetPrimAtPath(a).GetAttribute("xformOp:translate").Get()) \
            + (R0.T @ Ra) @ np.array([half, 0, 0])
        body(stage, h, 0.0005, tuple(hinge_pos), g.quat_wxyz(R0.T @ Rb), inertia=1e-8)
        joint(stage, f"{r}/joints/bend_{i:02d}", "angular", a, h, (half, 0, 0), axis="Z", rot0_wxyz=rel)
        joint(stage, f"{r}/joints/side_{i:02d}", "angular", h, b, (0, 0, 0), axis="Y", pos1=(-half, 0, 0),
              limits=(-g.SIDE_LIMIT, g.SIDE_LIMIT))
        no_collide(stage, a, b)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    build_drivetrain()
    build_chain()
    print("virtual front radius (m):", round(g.virtual_front_radius(), 4))
    print("CHAIN_POSE for task.py:", g.chain_pose_world())
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
