"""Step 0 for this task: can the YAM arms reach the stroller's buttons and handles at all?

    source <ARENA_DIR>/yam.env && python reach_check.py

For each key point it solves IK from each arm with a few gripper orientations and prints the best position error.
Anything above ~5 mm is out of reach. Needs yam_arena (no Isaac Sim, no scene).
"""

from __future__ import annotations

import math

import numpy as np

import geometry as g


def main() -> None:
    from yam_arena import ik, station
    from yam_arena.constants import READY
    from yam_arena.demo import grip_rotation

    def turn(axis, a):
        k = np.asarray(axis, float)
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K

    rotations = {"down": grip_rotation(0.0), "forward": turn([0, 1, 0], math.pi / 2) @ grip_rotation(0.0),
                 "side+y": turn([1, 0, 0], -math.pi / 2) @ grip_rotation(math.pi / 2),
                 "side-y": turn([1, 0, 0], math.pi / 2) @ grip_rotation(math.pi / 2)}
    print(f"{'point':16s} {'arm':6s} {'best orientation':16s} error_mm")
    for name, p in g.key_points_world().items():
        for side in ("left", "right"):
            best = (1e9, "")
            for rname, R in rotations.items():
                _, err, _ = ik.solve(np.asarray(p), ik.matrix_to_quat_xyzw(R), q_init=np.array(READY, float),
                                     base_pos=np.asarray(station.BASES[side], float), iters=300)
                best = min(best, (err * 1000, rname))
            print(f"{name:16s} {side:6s} {best[1]:16s} {best[0]:8.1f}")
    print("\nOut of reach if every error for a point is > 5 mm. Buttons need the left arm on button_left AND the right"
          "\narm on button_right. If they can't, this task can't be demonstrated at real scale: see README.md.")


if __name__ == "__main__":
    main()
