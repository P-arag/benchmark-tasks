"""Write the printer, cartridge and cover USD assets from geometry.py.

    python build_assets.py          # Isaac Sim's Python (pxr, PIL); no simulator needed

- assets/laser_printer.usda: fixed-base articulation. Root ``base`` (shell, cavity, rails, back wall); ``door`` on
  revolute ``door_hinge`` (about y at the bottom front edge; opening is negative); ``bump`` on prismatic
  ``detent`` (down into the floor against a spring).
- assets/cartridge_old.usda, assets/cartridge_new.usda: rigid bodies (body, handle bar, rib underneath, label).
- assets/cover.usda: the orange protective cover, a rigid body shaped like an inverted L over the cartridge's top
  front edge (top plate + front lip).

# CHECK (run): open each file and press Play. The old cartridge placed at its seat must stay home; nudging it out
# should need ~8-10 N (Physics Inspector force tool). The cover resting on the new cartridge must stay put when the
# cartridge is carried at 0.3 m/s; if it slides off, raise its friction (task.py) or deepen its front lip.
"""

from __future__ import annotations

from pathlib import Path

import geometry as g
from usdkit import articulation_root, body, box, capsule, joint, material, no_collide, quad, stage_for, text_png

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
GREY = (0.82, 0.82, 0.80)
DARK = (0.12, 0.12, 0.13)


def build_printer() -> None:
    stage = stage_for(OUT, "laser_printer")
    r = "/laser_printer"
    shell = material(stage, f"{r}/Looks/shell", GREY, roughness=0.6)
    dark = material(stage, f"{r}/Looks/dark", DARK, roughness=0.5)
    b = f"{r}/base"
    body(stage, b, 6.5)
    articulation_root(stage, b, fixed=True)
    hx, hy = g.D / 2, g.W / 2
    box(stage, f"{b}/floor", (g.D, g.W, g.FLOOR_Z), (0, 0, g.FLOOR_Z / 2), mat=shell)
    box(stage, f"{b}/roof", (g.D, g.W, g.H - g.CAV_TOP), (0, 0, (g.H + g.CAV_TOP) / 2), mat=shell)
    side_w = hy - g.CAV_Y
    for s, y in (("left", hy - side_w / 2), ("right", -hy + side_w / 2)):
        box(stage, f"{b}/side_{s}", (g.D, side_w, g.CAV_TOP - g.FLOOR_Z), (0, y, (g.CAV_TOP + g.FLOOR_Z) / 2),
            mat=shell)
    back_x = -hx + g.CAV_DEPTH
    box(stage, f"{b}/back", (hx - back_x, 2 * g.CAV_Y, g.CAV_TOP - g.FLOOR_Z),
        ((hx + back_x) / 2, 0, (g.CAV_TOP + g.FLOOR_Z) / 2), mat=dark)
    for s, y in (("left", g.CAV_Y - g.RAIL_W / 2), ("right", -g.CAV_Y + g.RAIL_W / 2)):
        box(stage, f"{b}/rail_{s}", (g.CAV_DEPTH, g.RAIL_W, g.RAIL_H),
            (-hx + g.CAV_DEPTH / 2, y, g.FLOOR_Z + g.RAIL_H / 2), mat=dark)
    # output tray recess on top (visual)
    box(stage, f"{b}/tray", (0.18, 0.22, 0.002), (0.02, 0, g.H + 0.001), collide=False, mat=dark)

    door = f"{r}/door"
    body(stage, door, 0.35, (-hx, 0, g.DOOR_HINGE_Z))
    box(stage, f"{door}/panel", (g.DOOR_T, g.W - 0.01, g.DOOR_H), (-g.DOOR_T / 2, 0, g.DOOR_H / 2), mat=shell)
    box(stage, f"{door}/lip", (0.012, 0.10, 0.006), (-g.DOOR_T - 0.006, 0, g.DOOR_H - 0.004), mat=dark)
    joint(stage, f"{r}/joints/door_hinge", "angular", b, door, (-hx, 0, g.DOOR_HINGE_Z), axis="Y",
          limits=(g.DOOR_OPEN, 0.0))
    no_collide(stage, door, b)

    bump = f"{r}/bump"
    bz = g.BUMP_TOP - g.BUMP_R
    body(stage, bump, 0.01, (g.BUMP_X, 0, bz))
    capsule(stage, f"{bump}/roller", g.BUMP_R, g.BUMP_LEN, (0, 0, 0), axis="Y", mat=dark)
    joint(stage, f"{r}/joints/detent", "linear", b, bump, (g.BUMP_X, 0, bz), axis="Z", limits=(-g.BUMP_TRAVEL, 0.0))
    no_collide(stage, bump, b)
    stage.GetRootLayer().Save()


def build_cartridge(name: str, mass: float, label_lines) -> None:
    stage = stage_for(OUT, name)
    r = f"/{name}"
    dark = material(stage, f"{r}/Looks/dark", DARK, roughness=0.5)
    body(stage, r, mass)                       # the root Xform is the rigid body
    box(stage, f"{r}/body", (g.C_D, g.C_L, g.C_H), (0, 0, 0), mat=dark)
    hx = g.C_D / 2
    # handle bar on the front face, on two posts
    hb = g.HANDLE
    box(stage, f"{r}/handle", hb, (-hx - g.HANDLE_OUT + hb[0] / 2, 0, 0.010), mat=dark)
    for s, y in (("l", hb[1] / 2 - 0.005), ("r", -hb[1] / 2 + 0.005)):
        box(stage, f"{r}/post_{s}", (g.HANDLE_OUT, 0.010, hb[2]), (-hx - g.HANDLE_OUT / 2, y, 0.010), mat=dark)
    box(stage, f"{r}/rib", g.RIB, (g.RIB_X, 0, -g.C_H / 2 - g.RIB[2] / 2), mat=dark)
    text_png(TEX / f"{name}_label.png", (900, 200), label_lines, bg=(235, 235, 232), font_px=60)
    lab = material(stage, f"{r}/Looks/label", (1, 1, 1), texture=f"./textures/{name}_label.png")
    quad(stage, f"{r}/label", 0.12, 0.027, (0.0, 0.07, g.C_H / 2 + 0.0005), "+Z", lab)
    stage.GetRootLayer().Save()


def build_cover() -> None:
    stage = stage_for(OUT, "cover")
    r = "/cover"
    orange = material(stage, f"{r}/Looks/orange", (0.95, 0.45, 0.05), roughness=0.5)
    body(stage, r, g.COVER_MASS)
    # in the cartridge's frame the cover's origin is the cartridge's top front edge; top plate over the top, lip down
    # the front face, and a pull tab sticking up
    box(stage, f"{r}/top", (0.030, g.COVER_L, 0.003), (0.015, 0, g.COVER_TOP - 0.0015), mat=orange)
    box(stage, f"{r}/spacer", (0.010, g.COVER_L, g.COVER_TOP - 0.003), (0.025, 0, (g.COVER_TOP - 0.003) / 2),
        mat=orange)
    box(stage, f"{r}/lip", (0.003, g.COVER_L, 0.025), (-0.0015, 0, g.COVER_TOP - 0.0125), mat=orange)
    box(stage, f"{r}/tab", (0.015, 0.030, 0.020), (0.015, 0, g.COVER_TOP + 0.010), mat=orange)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    build_printer()
    build_cartridge("cartridge_old", g.OLD_MASS, ["TONER CARTRIDGE", "USED"])
    build_cartridge("cartridge_new", g.NEW_MASS, ["TONER CARTRIDGE", "NEW"])
    build_cover()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
