# Put a dropped chain back on a child's bike: lay it on the chainring and turn the crank

A child's single-speed bike drivetrain at real size is clamped in a bench stand. The chain has dropped off the
chainring to the inside and hangs on the frame's bottom-bracket shell, still on the rear sprocket. The instruction:
"The chain has come off. Put it back on." One hand lays a few links onto the teeth, and the other turns the crank so
the ring draws the rest on.

> **STATUS: NOT RUN.** Written without the simulator. Every place that needs a run is marked `# CHECK (run)`.
> This is the hardest of the seven to make stable: a closed 70-link chain. Do steps 1–2 below **before** anything
> else; if the loop closure doesn't work in Isaac Lab, use the fallback described in `build_assets.py`.

| File | What it is |
|---|---|
| `task.json` | The task's details (scores and media to fill in) |
| `task.py` | Objects, the monitor (rollers seated on the ring / sprocket, crank turn while on), `outcome()`, `success()`, `score()` |
| `geometry.py` | Dimensions, plus the dropped chain's starting loop (computed so its length is exactly 70 pitches) |
| `build_assets.py` | Writes `assets/drivetrain.usda` and `assets/chain.usda` |
| `demo.py` | Open-loop scripted demo (and `--no-turn`, which must not reach 1.0) |
| `scenekit.py`, `demokit.py` | Program template helpers (unchanged) |
| `usdkit.py`, `taskkit.py` | USD-building and monitor helpers |

## Requirements and build

Isaac Lab-Arena with `yam_arena`, `inspect-robots`, Dream-1 client. Then `python build_assets.py`, which also prints
the virtual loop radius and link 0's pose (task.py computes the same pose from geometry.py).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The chain has come off. Put it back on." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/bike-chain/dream
python demo.py task.py demo_out
python demo.py task.py demo_noturn --no-turn --no-record     # must score < 1.0
```

## Scoring

| `outcome()` | Condition | Score |
|---|---|---|
| `done` | ≥ 10 rollers seated on the chainring, crank turned ≥ 360° while they stayed on, chain still 1 s | 1.0 |
| `on_not_turned` | ≥ 10 rollers seated at the end, no full turn with it on | 0.5 |
| `started` | 4–9 rollers seated at the end | 0.25 |
| `other` | Fewer, chain off the rear sprocket, nothing done | 0.0 |

Seated: the roller's centre lies within 3 mm of the pitch circle and within 3 mm of the ring's plane. The turn count
resets to zero whenever fewer than 10 rollers are seated, so spinning the crank before the chain is on earns nothing.

## Before you submit (in order)

1. Build, open `chain.usda` in Isaac Sim, press Play. The loop must hold together. If the excluded closing joint is
   rejected, fall back to anchoring link 0 to the sprocket (see `build_assets.py`).
2. Load the task with a zero policy: the chain must fall from its built loop onto the BB shell, stay on the sprocket
   and come to rest within ~2 s. Screenshot it for `setup.png`. If it misses the shell or comes off the sprocket,
   adjust `DROP_X` or `BB_SHELL_R`.
3. Hand-check the scorer: in a script, write the chain onto the ring (e.g. rebuild it around R_RING in the ring's
   plane) and print `_STATE['on_ring']`. It should be about 30 when fully on, and 0 at the start.
4. Tune `demo.py` (CHECK list at its top) to 1.0. A teleoperated demo is the fallback.
5. Run `--no-turn` (must be < 1.0), then Dream-1; fill in `scores`; videos and screenshot; upload; check the length
   limits; pre-review with `notes/judge-prompt.md`.

## Runtime, seeds and reset

- Run limit 2,400 steps. Runtime per trial: TODO (expect slow physics with ~140 chain bodies).
- Fixed layout (`object_jitter=0`), no seed.
- Reset: see `reset_notes`. The `before_reset` hook re-poses the crank. The chain returns to its built loop and falls
  again (CHECK in `task.py`).

## Third-party assets and licences

| Asset | Source | Licence |
|---|---|---|
| Drivetrain, stand and chain | Built from primitives by `build_assets.py` | Same as this repository |
| `scenekit.py`, `demokit.py` | Robocurve program template | As distributed by the program |
