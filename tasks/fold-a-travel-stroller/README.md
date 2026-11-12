# Fold a travel stroller: press both release buttons together, fold it until the storage latch catches

A compact travel stroller stands open on the table. The instruction is "Fold the stroller." Its fold only frees when
both red buttons at the main hinge are pressed in at the same time (a two-button child-safety release); the frame
then folds and a storage latch holds it.

> **STATUS: NOT RUN, AND PROBABLY NOT FEASIBLE AS IS.** A real compact stroller's hinge is about 45 cm and its
> handles about 95 cm above the table. The YAM arms reach about 20 cm above the table with a straight-down grip.
> **Run `reach_check.py` first** (one command, about a minute). Don't scale the stroller down to fit: reviewers mark
> that "Miniaturization", a final rejection (284 tasks were closed for it).

## If it's out of reach: the fallback

Keep the same skill (a two-sided simultaneous release, then fold until it latches) on an object at reachable height,
at real size:

- **Infant car-seat carrier handle:** press the two buttons at the handle's pivots together and swing the handle
  from carry to the "in car" position behind the shell until it clicks. The shell is ~40 cm tall, with the pivots
  ~25 cm up. Check the board first: a car-seat *buckling* task (#861) was rejected, but nothing about the handle.
- **Portable baby-bath or booster seat with two side latches:** same structure.

The code here carries over to either: swap the frame bodies in `build_assets.py`, keep `task.py`'s two-button lock
and latch logic, and rename.

| File | What it is |
|---|---|
| `reach_check.py` | Step 0: IK reachability of the buttons and handles from each arm |
| `task.json` | Task details (scores/media to fill) |
| `task.py` | The stroller; two-button release, re-lock, storage latch, tip-over tracking; `outcome()` etc. |
| `geometry.py` | Dimensions, fold angles, the reach-check points |
| `build_assets.py` | Writes `assets/stroller.usda` |
| `demo.py` | Open-loop demo; `--one-button` must score 0 |
| `scenekit.py`, `demokit.py`, `usdkit.py`, `taskkit.py` | Helpers |

## Run

```bash
python reach_check.py                     # step 0
python build_assets.py
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fold the stroller." --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/stroller/dream
python demo.py task.py demo_out
python demo.py task.py demo_one --one-button --no-record   # expect 0
```

## Scoring

| `outcome()` | Condition | Score |
|---|---|---|
| `folded` | Latched folded, within 30° of upright, still 1 s | 1.0 |
| `half` | Folded ≥ 20°, not latched | 0.5 |
| `unlocked` | Both buttons pressed together at some point, fold < 20° | 0.25 |
| `other` | Anything else; tipped past 60° at any time | 0.0 |

## Before you submit

1. `reach_check.py`. Stop here if the buttons are out of reach, and go to the fallback.
2. Build, Play: the open stroller must stand still, locked.
3. Tune the demo, confirm `--one-button` → 0, run Dream-1, fill `scores`, upload media, check the length limits,
   and pre-review with `notes/judge-prompt.md`.

## Third-party assets and licences

| Asset | Source | Licence |
|---|---|---|
| Stroller | Built from primitives by `build_assets.py` | Same as this repository |
| `scenekit.py`, `demokit.py` | Robocurve program template | As distributed by the program |
