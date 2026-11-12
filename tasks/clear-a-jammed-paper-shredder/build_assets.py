"""Write the shredder and paper-wad USD assets from geometry.py.

    python build_assets.py          # Isaac Sim's Python (pxr, PIL); no simulator needed

- assets/shredder.usda: fixed-base articulation. Root ``body`` (clear wastebasket, cutting head with a 5 mm feed slot
  in its top, two cutter shafts under the slot, switch track with REV / OFF / AUTO printed beside it); ``knob`` on
  prismatic ``switch_slide`` along y.
- assets/paper_wad.usda: a rigid body: a sheet sticking up out of the slot and a thicker folded part wedged in it.

# CHECK (run): press Play with the wad placed in the slot: it must sit on the cutters without being squeezed out.
# The slot walls are 0.25 mm clear of the folded part on each side.
"""

from __future__ import annotations

from pathlib import Path

import geometry as g
from usdkit import articulation_root, body, box, cylinder, joint, material, no_collide, quad, stage_for, text_png

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"


def build_shredder() -> None:
    stage = stage_for(OUT, "shredder")
    r = "/shredder"
    black = material(stage, f"{r}/Looks/black", (0.07, 0.07, 0.08), roughness=0.5)
    clear = material(stage, f"{r}/Looks/clear", (0.75, 0.80, 0.85), roughness=0.1)
    steel = material(stage, f"{r}/Looks/steel", (0.6, 0.6, 0.62), roughness=0.3, metallic=0.9)
    b = f"{r}/body"
    body(stage, b, 4.0)
    articulation_root(stage, b, fixed=True)
    # basket: floor and four walls (clear plastic, with some shredded paper visible inside: visual)
    bd, bw, bh, t = g.BASKET_D, g.BASKET_W, g.BASKET_H, 0.004
    box(stage, f"{b}/basket_floor", (bd, bw, t), (0, 0, t / 2), mat=clear)
    for n, size, pos in (("front", (t, bw, bh), (-bd / 2 + t / 2, 0, bh / 2)),
                         ("back", (t, bw, bh), (bd / 2 - t / 2, 0, bh / 2)),
                         ("left", (bd, t, bh), (0, bw / 2 - t / 2, bh / 2)),
                         ("right", (bd, t, bh), (0, -bw / 2 + t / 2, bh / 2))):
        box(stage, f"{b}/basket_{n}", size, pos, mat=clear)
    box(stage, f"{b}/shreds", (bd - 0.02, bw - 0.02, 0.03), (0, 0, 0.02), collide=False, color=(0.95, 0.95, 0.92))
    # head: underside, front/back/side walls, and a top in two halves either side of the slot
    hd, hw, hh, z0 = g.HEAD_D, g.HEAD_W, g.HEAD_H, g.BASKET_H
    box(stage, f"{b}/head_floor", (hd, hw, 0.006), (0, 0, z0 + 0.003), mat=black)
    for n, size, pos in (("front", (0.006, hw, hh), (-hd / 2 + 0.003, 0, z0 + hh / 2)),
                         ("back", (0.006, hw, hh), (hd / 2 - 0.003, 0, z0 + hh / 2)),
                         ("left", (hd, 0.006, hh), (0, hw / 2 - 0.003, z0 + hh / 2)),
                         ("right", (hd, 0.006, hh), (0, -hw / 2 + 0.003, z0 + hh / 2))):
        box(stage, f"{b}/head_{n}", size, pos, mat=black)
    top_t = 0.008
    half = (hd - g.SLOT_W) / 2
    for s, x in (("front", -(g.SLOT_W + half) / 2), ("back", (g.SLOT_W + half) / 2)):
        box(stage, f"{b}/top_{s}", (half, hw, top_t), (x, 0, g.TOTAL_H - top_t / 2), mat=black)
    # the slot only runs SLOT_LEN: close its two ends
    end_len = (hw - g.SLOT_LEN) / 2
    for s, y in (("l", g.SLOT_LEN / 2 + end_len / 2), ("r", -(g.SLOT_LEN / 2 + end_len / 2))):
        box(stage, f"{b}/slot_end_{s}", (g.SLOT_W + 0.002, end_len, top_t), (0, y, g.TOTAL_H - top_t / 2), mat=black)
    # throat walls from the slot down to the cutters
    depth = g.TOTAL_H - top_t - (g.CUTTER_Z + g.CUTTER_R)
    for s, x in (("front", -(g.SLOT_W / 2 + 0.002)), ("back", g.SLOT_W / 2 + 0.002)):
        box(stage, f"{b}/throat_{s}", (0.004, g.SLOT_LEN, depth), (x, 0, g.CUTTER_Z + g.CUTTER_R + depth / 2),
            mat=black)
    for s, x in (("front", -(g.CUTTER_R + g.CUTTER_GAP / 2)), ("back", g.CUTTER_R + g.CUTTER_GAP / 2)):
        cylinder(stage, f"{b}/cutter_{s}", g.CUTTER_R, g.SLOT_LEN, (x, 0, g.CUTTER_Z), axis="Y", mat=steel)
    # switch track and its printed positions
    sy = g.SWITCH_Y
    box(stage, f"{b}/track", (0.020, 0.040, 0.002), (g.SWITCH_X, sy, g.TOTAL_H + 0.001), collide=False,
        color=(0.2, 0.2, 0.22))
    text_png(TEX / "switch.png", (512, 128), ["REV    OFF    AUTO"], bg=(20, 20, 22), fg=(240, 240, 240), font_px=54)
    lab = material(stage, f"{r}/Looks/switch", (1, 1, 1), texture="./textures/switch.png")
    quad(stage, f"{b}/switch_label", 0.045, 0.011, (g.SWITCH_X + 0.016, sy, g.TOTAL_H + 0.0012), "+Z", lab)
    # CHECK (run): the label's text must read left-to-right REV, OFF, AUTO in the top camera with REV toward -y
    # (the robot's right). Flip the quad's uv or the list order if it comes out mirrored.

    k = f"{r}/knob"
    kz = g.TOTAL_H + g.KNOB[2] / 2
    body(stage, k, 0.01, (g.SWITCH_X, sy, kz))             # built at OFF: joint position 0 = OFF
    box(stage, f"{k}/cap", g.KNOB, (0, 0, 0), color=(0.85, 0.10, 0.08))
    joint(stage, f"{r}/joints/switch_slide", "linear", b, k, (g.SWITCH_X, sy, kz), axis="Y",
          limits=(g.POS["REV"], g.POS["AUTO"]))
    no_collide(stage, k, b)
    stage.GetRootLayer().Save()


def build_wad() -> None:
    stage = stage_for(OUT, "paper_wad")
    r = "/paper_wad"
    paper = material(stage, f"{r}/Looks/paper", (0.96, 0.96, 0.94), roughness=0.9)
    body(stage, r, g.WAD_MASS)
    # origin: at the slot's top, centred; the sheet sticks up, the folded part goes down to the cutters
    box(stage, f"{r}/sheet", (g.WAD_SHEET_T, g.WAD_W, g.WAD_SHOW), (0, 0, g.WAD_SHOW / 2), mat=paper)
    box(stage, f"{r}/jam", (g.WAD_JAM_T, g.WAD_W, g.WAD_JAM_DEPTH - 0.001), (0, 0, -(g.WAD_JAM_DEPTH - 0.001) / 2),
        mat=paper)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    build_shredder()
    build_wad()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
