"""Write the dog-form-with-collar and lead-with-snap USD assets from geometry.py.

    python build_assets.py          # Isaac Sim's Python (pxr); no simulator needed

- assets/dog_collar.usda: fixed-base articulation. Root ``dog`` (a seated plush-dog form built from primitives: body,
  chest, neck, head, muzzle, ears; a nylon collar band round the neck) and ``dring`` on revolute ``dring_hinge``
  (about y, at the collar's front-low point): a steel D-ring made of capsules, free to swing.
- assets/lead.usda: floating articulation. Root ``snap`` (swivel eye, barrel, hook as a chain of capsules), ``bolt``
  on prismatic ``bolt_slide`` along the barrel (negative = pulled back = open; a spring drives it shut) with its thumb
  trigger, and ``lead_00..09`` webbing links hinged behind the eye.

# CHECK (run): (1) open dog_collar.usda and swing the D-ring by hand: it must hang down at rest and swing +-90 degrees
# without catching on the collar. (2) In lead.usda pull the bolt back 10 mm: the gap between the bolt's tip and the
# hook's tip must open to ~9 mm (enough for the 3.5 mm D-ring wire). (3) If a plush-dog mesh with a clear licence is
# available in the Arena asset library, use it for the visuals and keep these primitives as colliders.
"""

from __future__ import annotations

import math
from pathlib import Path

import geometry as g
from usdkit import (articulation_root, body, box, capsule, cylinder, joint, material, no_collide, quat_axis_angle,
                    sphere, stage_for)

OUT = Path(__file__).resolve().parent / "assets"
STEEL = (0.70, 0.71, 0.73)


def wire(stage, path, p0, p1, radius, mat):
    """A capsule from p0 to p1 (both in the parent's frame), in the xz plane or along y."""
    p0, p1 = list(p0), list(p1)
    d = [p1[i] - p0[i] for i in range(3)]
    length = math.sqrt(sum(v * v for v in d))
    mid = tuple((p0[i] + p1[i]) / 2 for i in range(3))
    if abs(d[1]) > 1e-9 and abs(d[0]) < 1e-9 and abs(d[2]) < 1e-9:
        return capsule(stage, path, radius, max(1e-4, length), mid, axis="Y", mat=mat)
    if abs(d[0]) < 1e-9 and abs(d[2]) > 1e-9 and abs(d[1]) < 1e-9:
        return capsule(stage, path, radius, max(1e-4, length), mid, axis="Z", mat=mat)
    if abs(d[0]) < 1e-9:                         # in the yz plane: tilt an x... use a Y capsule rotated about x
        ang = math.atan2(d[2], d[1])
        return capsule(stage, path, radius, max(1e-4, length), mid, axis="Y", mat=mat,
                       rot_wxyz=quat_axis_angle((1, 0, 0), ang))
    ang = math.atan2(d[2], d[0])                 # in the xz plane: an X capsule rotated about y
    return capsule(stage, path, radius, max(1e-4, length), mid, axis="X", mat=mat,
                   rot_wxyz=quat_axis_angle((0, 1, 0), -ang))


def build_dog() -> None:
    stage = stage_for(OUT, "dog_collar")
    r = "/dog_collar"
    fur = material(stage, f"{r}/Looks/fur", (0.72, 0.55, 0.36), roughness=0.95)
    dark = material(stage, f"{r}/Looks/dark", (0.15, 0.10, 0.08), roughness=0.9)
    red = material(stage, f"{r}/Looks/collar", (0.70, 0.08, 0.10), roughness=0.7)
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.25, metallic=0.95)
    d = f"{r}/dog"
    body(stage, d, 3.0)
    articulation_root(stage, d, fixed=True)
    # origin: the form's footprint centre on the table; facing -x (the arms)
    nx = 0.0
    sphere(stage, f"{d}/haunch", 0.085, (0.06, 0, 0.075), mat=fur)
    cylinder(stage, f"{d}/chest", 0.065, 0.12, (0.0, 0, 0.065), mat=fur)
    cylinder(stage, f"{d}/neck", g.NECK_R, 0.09, (nx, 0, g.NECK_Z), mat=fur)
    sphere(stage, f"{d}/head", 0.060, (nx, 0, g.NECK_Z + 0.075), mat=fur)
    capsule(stage, f"{d}/muzzle", 0.025, 0.03, (nx - 0.065, 0, g.NECK_Z + 0.06), axis="X", mat=fur)
    sphere(stage, f"{d}/nose", 0.010, (nx - 0.095, 0, g.NECK_Z + 0.065), collide=False, mat=dark)
    for s, y in (("l", 0.045), ("r", -0.045)):
        box(stage, f"{d}/ear_{s}", (0.03, 0.01, 0.05), (nx + 0.01, y, g.NECK_Z + 0.105), collide=False, mat=dark)
        capsule(stage, f"{d}/leg_{s}", 0.018, 0.08, (-0.04, y * 0.8, 0.04), mat=fur)
    cylinder(stage, f"{d}/collar", g.NECK_R + 0.004, g.COLLAR_W, (nx, 0, g.DRING_HINGE[2]), mat=red)

    dr = f"{r}/dring"
    hinge_local = (g.DRING_HINGE[0] - g.DOG_X, g.DRING_HINGE[1] - g.DOG_Y, g.DRING_HINGE[2])
    body(stage, dr, 0.012, hinge_local)
    pts = [tuple(p) for p in g.d_loop(12)]
    for k in range(len(pts)):
        wire(stage, f"{dr}/w{k:02d}", pts[k], pts[(k + 1) % len(pts)], g.D_WIRE_R, steel)
    joint(stage, f"{r}/joints/dring_hinge", "angular", d, dr, hinge_local, axis="Y",
          limits=(-math.radians(100), math.radians(100)))
    no_collide(stage, dr, d)
    # CHECK (run): with dring/dog collisions filtered the D-ring can swing through the neck. Limits of +-100 degrees
    # keep it out in front; if it still clips into the form, re-enable the collision and move the hinge 3 mm out.
    stage.GetRootLayer().Save()


def build_lead() -> None:
    stage = stage_for(OUT, "lead")
    r = "/lead"
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.25, metallic=0.95)
    nylon = material(stage, f"{r}/Looks/nylon", (0.10, 0.25, 0.65), roughness=0.8)
    s = f"{r}/snap"
    body(stage, s, g.SNAP_MASS)
    articulation_root(stage, s, fixed=False)
    capsule(stage, f"{s}/barrel", g.BARREL_R, g.BARREL - 2 * g.BARREL_R, (-g.BARREL / 2, 0, -0.001), axis="X",
            mat=steel)
    # swivel eye behind the barrel (as four short wires round a square-ish loop)
    ex = -g.BARREL - g.EYE_R
    eye = [(ex - g.EYE_R, 0, 0), (ex, 0, g.EYE_R), (ex + g.EYE_R, 0, 0), (ex, 0, -g.EYE_R)]
    for k in range(4):
        wire(stage, f"{s}/eye{k}", eye[k], eye[(k + 1) % 4], 0.0018, steel)
    pts = [(x, 0.0, z) for x, z in g.HOOK_XZ]
    for k in range(len(pts) - 1):
        wire(stage, f"{s}/hook{k}", pts[k], pts[k + 1], g.WIRE_R, steel)

    b = f"{r}/bolt"
    body(stage, b, 0.004, (g.BOLT_CLOSED_TIP - g.BOLT_LEN / 2, 0, g.BOLT_Z))
    capsule(stage, f"{b}/pin", 0.0019, g.BOLT_LEN - 0.0038, (0, 0, 0), axis="X", mat=steel)
    box(stage, f"{b}/trigger", g.TRIGGER, (-g.BOLT_LEN / 2 - 0.004, 0, g.TRIGGER[2] / 2 + 0.003), mat=steel)
    joint(stage, f"{r}/joints/bolt_slide", "linear", s, b, (g.BOLT_CLOSED_TIP - g.BOLT_LEN / 2, 0, g.BOLT_Z),
          axis="X", limits=(-g.BOLT_TRAVEL, 0.0))
    no_collide(stage, b, s)
    # CHECK (run): the trigger must be reachable from above with the snap held by its barrel; if the barrel grip
    # covers it, move the trigger 1 cm further back (toward the eye) along a slot.

    prev, x = s, -g.BARREL - 2 * g.EYE_R
    for k in range(g.LEAD_LINKS):
        link = f"{r}/lead_{k:02d}"
        body(stage, link, g.LEAD_LINK_MASS, (x - g.LEAD_LINK / 2, 0, 0))
        box(stage, f"{link}/web", (g.LEAD_LINK, g.LEAD_T, g.LEAD_W), (0, 0, 0), mat=nylon)
        pos0 = (x, 0, 0) if k == 0 else (-g.LEAD_LINK / 2, 0, 0)
        joint(stage, f"{r}/joints/lead_{k:02d}", "angular", prev, link, pos0, axis="Z", pos1=(g.LEAD_LINK / 2, 0, 0),
              limits=(-math.radians(150), math.radians(150)))
        no_collide(stage, link, prev)
        prev, x = link, x - g.LEAD_LINK
    # the last link: a hand loop (visual only)
    cylinder(stage, f"{prev}/loop", 0.03, 0.002, (-g.LEAD_LINK / 2 - 0.03, 0, 0), axis="Y", collide=False, mat=nylon)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    build_dog()
    build_lead()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
