# Open a ring-pull tin of soup without spilling it

A full tin of soup with a ring-pull lid stands free on the table. The instruction is "Open the can." The robot has
to lift the ring until the seal cracks, then peel the lid back off the opening, while the other hand keeps the
free-standing tin still and level. Once open, the soup sits 1 cm below the rim, so tilting the tin past ~15° spills
it.

> **STATUS: NOT RUN.** Written without the simulator; search the code for `# CHECK (run)`. The main risk is
> feasibility: getting a YAM fingertip under a ring tab. Check that first (demo step 2).

| File | What it is |
|---|---|
| `task.json` | Task details (scores/media to fill) |
| `task.py` | The tin, the crack/peel monitor, spill and knock-over tracking, `outcome()`, `success()`, `score()` |
| `geometry.py` | Dimensions and the spill tilt (derived from the 1 cm headspace) |
| `build_assets.py` | Writes `assets/soup_can.usda` and its label texture |
| `demo.py` | Open-loop demo; `--no-hold` (should spill → 0) and `--yank` |
| `scenekit.py`, `demokit.py`, `usdkit.py`, `taskkit.py` | Helpers (template + ours) |

## Run a trial

```bash
python build_assets.py
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can." --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/ring-pull/dream
python demo.py task.py demo_out
python demo.py task.py demo_nohold --no-hold --no-record    # expect 0 (spill)
```

## Scoring

| `outcome()` | Condition | Score |
|---|---|---|
| `open` | Lid ≥ 100° back, no spill, tin still 1 s | 1.0 |
| `half` | Lid ≥ 45° back, no spill | 0.5 |
| `cracked` | Seal cracked (tab lifted ≥ 55°), lid < 45°, no spill | 0.25 |
| `other` | Nothing done, any spill after the crack, or the tin tilted past 60° at any time | 0.0 |

A spill or a knock-over is permanent: standing the tin back up doesn't clear it.

## Before you submit (in order)

1. Build; open in Isaac Sim; the tin must stand still.
2. Feasibility: can a YAM fingertip get under the ring end? Try demo step 2 alone. If not, cut a real recess into
   the lid collider (see the end of `build_assets.py`). Don't make the tab bigger than a real one: that would draw a
   "miniaturization/simplification" flag the other way.
3. Calibrate the real-world numbers: measure a real tin's crack force (luggage scale on the ring) and peel force,
   then set `TAB_K` / `PEEL_DAMPING` to match, and say so in `fidelity`.
4. Tune the demo to 1.0; check `--no-hold` gives 0; run Dream-1; fill `scores`; media; length limits;
   pre-review with `notes/judge-prompt.md`.

## Runtime, seeds and reset

Run limit 2,400 steps; runtime TODO; fixed layout, no seed. Real trials need a new tin each time (`reset_notes`).
Safety for real trials: the torn lid edge is sharp. Have the operator handle it with gloves, and put a drip tray
under the tin.

## Third-party assets and licences

| Asset | Source | Licence |
|---|---|---|
| Tin, lid, tab, label | Built from primitives by `build_assets.py` | Same as this repository |
| DejaVu Sans Bold (label text) | dejavu-fonts.github.io | Bitstream Vera / DejaVu licence |
| `scenekit.py`, `demokit.py` | Robocurve program template | As distributed by the program |
