"""Write the printer and paper-roll USD assets from geometry.py.

    python build_assets.py          # Isaac Sim's Python (pxr, PIL); the simulator itself is not needed

- assets/receipt_printer.usda: a fixed-base articulation. Root ``base`` (shell, front wall = print head, paper
  bucket cradle, deck strips, bucket sticker); ``lid`` on revolute ``lid_hinge`` (about y, at the back top edge)
  carrying the platen roller; ``button`` on prismatic ``button_press`` (down 4 mm).
- assets/paper_roll.usda: a floating articulation. Root ``roll`` (paper cylinder, axis along local y) and a chain of
  ``tail_00..tail_11`` strip links on revolute hinges about y. The first link leaves the TOP of the roll toward +x.

# CHECK (run): after building, open both files in Isaac Sim and press Play:
#   - the empty printer must sit still (no jitter of the lid at 0 while latched),
#   - the roll dropped from 5 cm must come to rest with its tail draped, not exploding or vibrating. If the tail
#     jitters, raise solver_iters in task.py, raise TAIL_T to 2 mm or TAIL_LINK_MASS to 4 g.
"""

from __future__ import annotations

from pathlib import Path

import geometry as g
from usdkit import (articulation_root, body, box, cylinder, joint, material, no_collide, quad, stage_for, text_png,
                    arc_of_boxes)

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"

SHELL = (0.13, 0.13, 0.14)
LID_C = (0.17, 0.17, 0.18)
PAPER = (0.97, 0.97, 0.95)


def build_printer() -> None:
    stage = stage_for(OUT, "receipt_printer")
    r = "/receipt_printer"
    shell = material(stage, f"{r}/Looks/shell", SHELL, roughness=0.6)
    lidm = material(stage, f"{r}/Looks/lid", LID_C, roughness=0.5)
    dark = material(stage, f"{r}/Looks/dark", (0.05, 0.05, 0.05), roughness=0.4)
    metal = material(stage, f"{r}/Looks/metal", (0.6, 0.6, 0.62), roughness=0.3, metallic=0.8)
    bucket = material(stage, f"{r}/Looks/bucket", (0.22, 0.22, 0.24), roughness=0.5)

    b = f"{r}/base"
    body(stage, b, 1.6)
    articulation_root(stage, b, fixed=True)
    hx, hy = g.D / 2, g.W / 2
    # shell: floor, back, sides, front wall (its top edge is the print head)
    box(stage, f"{b}/floor", (g.D, g.W, g.WALL), (0, 0, g.WALL / 2), mat=shell)
    box(stage, f"{b}/back", (g.WALL, g.W, g.H), (hx - g.WALL / 2, 0, g.H / 2), mat=shell)
    for s, y in (("left", hy - g.WALL / 2), ("right", -hy + g.WALL / 2)):
        box(stage, f"{b}/side_{s}", (g.D, g.WALL, g.H), (0, y, g.H / 2), mat=shell)
    box(stage, f"{b}/front", (g.WALL, g.W, g.HEAD_TOP), (g.FRONT_X + g.WALL / 2, 0, g.HEAD_TOP / 2), mat=shell)
    box(stage, f"{b}/head", (0.010, g.LID_W, 0.004), (g.FRONT_X + 0.011, 0, g.HEAD_TOP - 0.002), mat=dark)
    # deck strips either side of the lid (the button sits on the right one)
    for s, y in (("left", hy - 0.0075), ("right", -hy + 0.0075)):
        box(stage, f"{b}/deck_{s}", (g.D, 0.015, g.WALL), (0, y, g.H - g.WALL / 2), mat=shell)
    # paper bucket: a half-round cradle of flat segments under the roll, with side walls
    arc_of_boxes(stage, f"{b}/cradle", g.BUCKET_R + 0.002, 3.4, 6.0, 14, (0.004, g.BUCKET_W, 0.020),
                 0.0, (0.2, 0.2, 0.2), axis_normal="Y", mat=bucket, center=(g.BUCKET_X, 0.0, g.BUCKET_Z))
    # CHECK (run): look at the cradle from the side: 14 flat segments forming the lower half of a 4.55 cm circle
    # centred at (BUCKET_X, BUCKET_Z). If they stand radially instead of tangentially, flip the sign of the angle in
    # usdkit.arc_of_boxes.
    for s, y in (("left", g.BUCKET_W / 2 + 0.002), ("right", -g.BUCKET_W / 2 - 0.002)):
        box(stage, f"{b}/bucket_{s}", (0.10, 0.004, 0.09), (g.BUCKET_X, y, g.BUCKET_Z - 0.01), mat=bucket)
    # the bucket sticker: paper leaves the TOP of the roll toward the front (printed arrow)
    text_png(TEX / "sticker.png", (512, 256), ["PAPER", "<- over the top"], bg=(250, 210, 40), font_px=56)
    sticker = material(stage, f"{r}/Looks/sticker", (1, 1, 1), texture="./textures/sticker.png")
    quad(stage, f"{b}/sticker", 0.06, 0.03, (g.BUCKET_X, -g.BUCKET_W / 2 + 0.0005, g.BUCKET_Z + 0.02), "+Y", sticker)
    # the tear bar: a thin serrated metal strip along the front edge (visual)
    box(stage, f"{b}/tear_bar", (0.002, g.LID_W, 0.006), (g.FRONT_X - 0.001, 0, g.HEAD_TOP + 0.004), collide=False,
        mat=metal)

    # lid, hinged at the back top edge; its origin is the hinge line
    lid = f"{r}/lid"
    body(stage, lid, g.LID_MASS, (g.HINGE_X, 0, g.HINGE_Z))
    box(stage, f"{lid}/plate", (g.LID_L, g.LID_W, g.LID_T), (-g.LID_L / 2, 0, g.LID_T / 2), mat=lidm)
    drop = g.H - g.HEAD_TOP - g.SLOT              # the front lip hangs down to just above the paper path
    box(stage, f"{lid}/lip", (0.006, g.LID_W, drop), (-g.LID_L + 0.003, 0, -drop / 2), mat=lidm)
    cylinder(stage, f"{lid}/platen", g.PLATEN_R, g.LID_W - 0.01, g.PLATEN_LOCAL, axis="Y", mat=dark)
    text_png(TEX / "lid_logo.png", (512, 128), ["RECEIPT 80"], bg=(44, 44, 46), fg=(200, 200, 200))
    logo = material(stage, f"{r}/Looks/logo", (1, 1, 1), texture="./textures/lid_logo.png")
    quad(stage, f"{lid}/logo", 0.06, 0.015, (-0.13, 0, g.LID_T + 0.0005), "+Z", logo)
    joint(stage, f"{r}/joints/lid_hinge", "angular", b, lid, (g.HINGE_X, 0, g.HINGE_Z), axis="Y",
          limits=(0.0, g.LID_MAX))
    no_collide(stage, lid, b)                      # the lid's lip sits inside the shell's outline
    # CHECK (run): with lid/base collisions filtered, the shut lid rests only on its joint limit (0); that is what
    # we want (the latch is the limit). Make sure the paper strip still collides with BOTH lid and base.

    # release button, on the right deck strip
    btn = f"{r}/button"
    body(stage, btn, 0.005, (g.BUTTON_X, g.BUTTON_Y, g.H + g.BUTTON_H / 2))
    cylinder(stage, f"{btn}/cap", g.BUTTON_R, g.BUTTON_H, (0, 0, 0), mat=dark)
    joint(stage, f"{r}/joints/button_press", "linear", b, btn, (g.BUTTON_X, g.BUTTON_Y, g.H + g.BUTTON_H / 2),
          axis="Z", limits=(-g.BUTTON_TRAVEL, 0.0))
    no_collide(stage, btn, b)
    stage.GetRootLayer().Save()


def build_roll() -> None:
    stage = stage_for(OUT, "paper_roll")
    r = "/paper_roll"
    paper = material(stage, f"{r}/Looks/paper", PAPER, roughness=0.8)
    core = material(stage, f"{r}/Looks/core", (0.55, 0.42, 0.28), roughness=0.9)
    roll = f"{r}/roll"
    body(stage, roll, g.ROLL_MASS)
    articulation_root(stage, roll, fixed=False)
    cylinder(stage, f"{roll}/paper", g.ROLL_R, g.ROLL_W, (0, 0, 0), axis="Y", mat=paper)
    for s, y in (("left", g.ROLL_W / 2 + 0.0002), ("right", -g.ROLL_W / 2 - 0.0002)):
        cylinder(stage, f"{roll}/core_{s}", g.CORE_R, 0.0004, (0, y, 0), axis="Y", collide=False, mat=core)
    # the free end: leaves the top of the roll toward +x, link by link
    prev, x = roll, 0.0
    for k in range(g.TAIL_N):
        link = f"{r}/tail_{k:02d}"
        body(stage, link, g.TAIL_LINK_MASS, (x + g.TAIL_LINK / 2, 0, g.ROLL_R + g.TAIL_T / 2))
        box(stage, f"{link}/strip", (g.TAIL_LINK, g.TAIL_W, g.TAIL_T), (0, 0, 0), mat=paper)
        pos0 = (0.0, 0, g.ROLL_R + g.TAIL_T / 2) if k == 0 else (g.TAIL_LINK / 2, 0, 0)
        joint(stage, f"{r}/joints/tail_{k:02d}", "angular", prev, link, pos0, axis="Y",
              pos1=(-g.TAIL_LINK / 2, 0, 0), limits=(-g.TAIL_LIMIT, g.TAIL_LIMIT))
        if k == 0:
            no_collide(stage, link, roll)
        prev, x = link, x + g.TAIL_LINK
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    build_printer()
    build_roll()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
