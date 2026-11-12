"""Write the soup-tin USD asset from geometry.py.

    python build_assets.py          # Isaac Sim's Python (pxr, PIL); no simulator needed

assets/soup_can.usda: a FLOATING articulation (the tin stands free and can be pushed, dragged or knocked over).
Root ``can`` (body, seam ring, label, soup surface under the lid); ``lid`` on revolute ``lid_peel`` about y at its
far (+x) edge, locked by task.py until the score cracks; ``tab`` on revolute ``tab_lever`` about y at the rivet.
Positive angles lift the ring end / the lid's near edge.

# CHECK (run): open in Isaac Sim, press Play: the tin must stand still; then drag the ring up in the Physics
# Inspector (or set the tab joint to 60 degrees) and watch the lid unlock (only with task.py running).
"""

from __future__ import annotations

from pathlib import Path

import geometry as g
from usdkit import (articulation_root, body, box, cylinder, joint, material, no_collide, quad, stage_for, text_png)

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
TIN = (0.78, 0.78, 0.80)


def build() -> None:
    stage = stage_for(OUT, "soup_can")
    r = "/soup_can"
    tin = material(stage, f"{r}/Looks/tin", TIN, roughness=0.25, metallic=0.9)
    soup = material(stage, f"{r}/Looks/soup", (0.70, 0.15, 0.08), roughness=0.3)
    text_png(TEX / "label.png", (1400, 520), ["TOMATO SOUP", "400 g", "EASY OPEN"], bg=(190, 30, 30),
             fg=(255, 245, 230), font_px=110)
    label = material(stage, f"{r}/Looks/label", (1, 1, 1), texture="./textures/label.png")

    c = f"{r}/can"
    body(stage, c, g.CAN_MASS)
    articulation_root(stage, c, fixed=False)
    cylinder(stage, f"{c}/body", g.CAN_R, g.LID_Z - 0.001, (0, 0, (g.LID_Z - 0.001) / 2), mat=tin)
    # the seam: a ring standing 3 mm proud of the lid round the rim (four boxes would do; a thin tube is visual)
    cylinder(stage, f"{c}/seam", g.CAN_R + 0.0008, g.RIM_H + 0.002, (0, 0, g.CAN_H - (g.RIM_H + 0.002) / 2 + 0.001),
             collide=False, mat=tin)
    # the soup's surface, visible once the lid is back (visual only; see fidelity)
    cylinder(stage, f"{c}/soup", g.LID_R - 0.001, 0.001, (0, 0, g.CAN_H - g.FILL_GAP), collide=False, mat=soup)
    # label wrapped round the front: approximated by a flat quad facing the arms (CHECK (run): replace with a
    # band like child-resistant-cap's build_assets.band() if it looks flat in the camera)
    quad(stage, f"{c}/label", 0.07, 0.07, (-g.CAN_R - 0.0005, 0, 0.05), "-X", label)

    lid = f"{r}/lid"
    body(stage, lid, 0.008, (g.HINGE_X, 0, g.LID_Z))
    # the lid disc, offset so its far edge is on the hinge line
    cylinder(stage, f"{lid}/disc", g.LID_R, g.LID_T, (-g.HINGE_X, 0, g.LID_T / 2), mat=tin)
    joint(stage, f"{r}/joints/lid_peel", "angular", c, lid, (g.HINGE_X, 0, g.LID_Z), axis="Y",
          limits=(0.0, g.PEEL_MAX))
    no_collide(stage, lid, c)

    tab = f"{r}/tab"
    rivet = (g.RIVET_X - g.HINGE_X, 0, g.LID_T)            # in the lid's frame
    body(stage, tab, 0.002, (g.RIVET_X, 0, g.LID_Z + g.LID_T))
    ring_len = g.TAB_L - g.TAB_NOSE
    # the tab lies flat; its ring end (toward -x) is raised a little and has a dimple under it in the lid (visual)
    box(stage, f"{tab}/plate", (g.TAB_L, g.TAB_W, g.TAB_T), (-(ring_len - g.TAB_NOSE) / 2, 0, g.TAB_T / 2 + 0.0003),
        mat=tin)
    box(stage, f"{tab}/ring_lip", (0.004, g.TAB_W, 0.0015), (-ring_len + 0.002, 0, g.RING_RAISE), mat=tin)
    cylinder(stage, f"{lid}/dimple", 0.008, 0.0005, (g.RIVET_X - g.HINGE_X - ring_len + 0.006, 0, g.LID_T + 0.0002),
             collide=False, color=(0.4, 0.4, 0.42))
    joint(stage, f"{r}/joints/tab_lever", "angular", lid, tab, rivet, axis="Y", limits=(0.0, g.TAB_MAX))
    no_collide(stage, tab, lid)
    # CHECK (run): the finger dimple is only visual here. If no gripper finger can get under the ring end, cut the
    # lid's collider: replace the single lid disc by two boxes + a disc leaving a RECESS_DEPTH-deep gap under the
    # ring end, and re-run the demo.
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    build()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
