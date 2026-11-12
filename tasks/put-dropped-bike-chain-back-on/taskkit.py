"""taskkit: helpers shared by task.py's monitor and scorer (pure Python at import; Isaac imports are lazy).

- :func:`write_limits` / :func:`write_damping`: change an articulation joint at run time (latches, locks).
- :func:`fingertips`: both grippers' finger link positions, for keep-out zones.
- :class:`Still`: "at rest for N seconds", robust to PhysX putting settled bodies to sleep.
- :func:`tilt`: angle between a body's z axis and world up.
"""

from __future__ import annotations

import math

import numpy as np

CONTROL_HZ = 20

# CHECK (run): print(robot_links(env)["left"].keys()) once and put the real finger-pad link names here. The YAM
# URDF in yam_arena names them something like "left_finger"/"right_finger" or "link_7"/"link_8".
FINGER_LINK_KEYS = ("finger", "pad", "tip")
FINGER_PAD_REACH = 0.013  # yam_arena.constants.PAD_TIP_PAST_TCP: the pads' tips lie this far past the link origin


def write_limits(environment, asset_name: str, joint: str, lo: float, hi: float) -> None:
    """Set one joint's position limits (rad or m) in the running simulation."""
    import torch

    asset = environment.unwrapped.scene[asset_name]
    ids = asset.find_joints([joint])[0]
    asset.write_joint_position_limit_to_sim_index(
        limits=torch.tensor([[[lo, hi]]], dtype=torch.float32, device=asset.device), joint_ids=ids,
        warn_limit_violation=False)


def write_damping(environment, asset_name: str, joint: str, damping: float) -> None:
    import torch

    asset = environment.unwrapped.scene[asset_name]
    ids = asset.find_joints([joint])[0]
    asset.write_joint_damping_to_sim_index(
        damping=torch.tensor([[damping]], dtype=torch.float32, device=asset.device), joint_ids=ids)


def apply_force(environment, asset_name: str, force_xyz, body: str | None = None) -> None:
    """A constant external force (N, world frame) on a rigid body (or one link of an articulation) until changed.
    CHECK (run): Isaac Lab's API name has changed between versions (set_external_force_and_torque /
    permanent_wrench_composer); confirm it on 0.3.1 and that the force is applied every physics substep."""
    import torch

    asset = environment.unwrapped.scene[asset_name]
    if body is None:
        ids = [0]
    else:
        ids = asset.find_bodies([body])[0]
    f = torch.tensor([[list(force_xyz)] * len(ids)], dtype=torch.float32, device=asset.device)
    asset.set_external_force_and_torque(forces=f, torques=torch.zeros_like(f), body_ids=ids)


def body_poses(environment, asset_name: str) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Every link of an articulation (or the one body of a rigid object): name -> (pos, quat_xyzw), station frame.
    CHECK (run): Isaac Lab stores quaternions as (w, x, y, z) in ``body_quat_w``; this converts to xyzw."""
    base = environment.unwrapped
    asset = base.scene[asset_name]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    pos = asset.data.body_pos_w[0].detach().cpu().numpy() - origin
    quat_wxyz = asset.data.body_quat_w[0].detach().cpu().numpy()
    return {n: (pos[i], np.r_[quat_wxyz[i, 1:], quat_wxyz[i, 0]]) for i, n in enumerate(asset.body_names)}


def fingertips(environment) -> list[np.ndarray]:
    """Positions (station frame) of every finger link of both arms."""
    from scenekit import robot_links

    pts = []
    for side, links in robot_links(environment).items():
        for name, p in links.items():
            if any(k in name.lower() for k in FINGER_LINK_KEYS):
                pts.append(np.asarray(p, float))
    return pts


def in_box(p, lo, hi, margin: float = 0.0) -> bool:
    p, lo, hi = (np.asarray(v, float) for v in (p, lo, hi))
    return bool(np.all(p >= lo - margin) and np.all(p <= hi + margin))


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt(quat_xyzw) -> float:
    """Radians between the body's z axis and world up."""
    return math.acos(max(-1.0, min(1.0, rotation(quat_xyzw)[2, 2])))


class Still:
    """Counts how long a body has been at rest (under 1 cm/s and 0.5 rad/s, or not moved since the previous control
    step: PhysX can put a settled body to sleep while it still reports a small velocity)."""

    def __init__(self):
        self.prev: dict = {}
        self.held: dict = {}

    def reset(self) -> None:
        self.prev.clear()
        self.held.clear()

    def update(self, name: str, pos, quat_xyzw, lin_vel, ang_vel) -> float:
        now = np.r_[np.asarray(pos, float), np.asarray(quat_xyzw, float)]
        last = self.prev.get(name)
        self.prev[name] = now
        slow = np.linalg.norm(lin_vel) < 0.01 and np.linalg.norm(ang_vel) < 0.5
        unmoved = last is not None and np.linalg.norm(now[:3] - last[:3]) < 0.0005 and np.abs(now[3:] - last[3:]).max() < 0.01
        self.held[name] = self.held.get(name, 0.0) + 1.0 / CONTROL_HZ if (slow or unmoved) else 0.0
        return self.held[name]
