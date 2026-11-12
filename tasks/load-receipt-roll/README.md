# Load a new paper roll into a receipt printer the right way round and latch its lid

A desktop thermal receipt printer stands on the table with its lid shut over an empty paper bucket; a new roll
lies beside it, wound so that dropping it straight in would put it in backwards. The instruction only says "Load
the new paper roll so the printer can print." Doing that takes a lid release, a 180-degree turn of the roll (read
from the sticker in the bucket), leaving the free end out over the tear bar, and pressing the lid shut until it
latches.

> **STATUS: NOT RUN.** This folder was written without access to the simulator. Every place that needs a run to
> confirm or tune is marked `# CHECK (run)` in the code (search for it). Work through **Before you submit** below,
> top to bottom.

| File | What it is |
|---|---|
| `task.json` | The task's details (scores and media still to fill in) |
| `task.py` | The task file: objects, the latch monitor, `outcome()`, `success()`, `score()` |
| `geometry.py` | Every dimension, shared by the other files |
| `build_assets.py` | Writes `assets/receipt_printer.usda` and `assets/paper_roll.usda` (and the sticker/logo textures) |
| `demo.py` | Scripted open-loop full-score demo (and `--no-turn`, the backwards drop, which scores 0) |
| `scenekit.py`, `demokit.py` | The program template's helpers for USD bodies/articulations and scripted demos (unchanged) |
| `usdkit.py`, `taskkit.py` | Small helpers for building the USD assets and for the monitor (latches, stillness, link poses) |

## Requirements

The Isaac Lab-Arena environment from the program's Brev launchable, with the `yam_arena` embodiment,
`inspect-robots` and the Dream-1 client. Build the assets once with Isaac Sim's Python (needs `pxr`, `PIL` and the
DejaVu fonts):

```bash
cd tasks/load-receipt-roll && python build_assets.py
```

## Run a trial

From this folder, after `source <ARENA_DIR>/yam.env`:

```bash
# Dream-1, at the run limit
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the new paper roll so the printer can print." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/load-receipt-roll/dream

# Scripted demo (writes demo_*.mp4, setup.png, result.json)
python demo.py task.py demo_out

# Negative check: the roll dropped in without turning it must score 0 ("reversed")
python demo.py task.py demo_noturn --no-turn --no-record

# Dream-1 videos
for log in ~/yam-eval/load-receipt-roll/*/*.json; do inspect-robots video "$log"; done
```

## Scoring

`task_score` gives 1.0 once `success()` has held for 1 s, otherwise `score()` at the end.

| `outcome()` | Condition | Score |
|---|---|---|
| `done` | In the bucket, right way round, lid latched, free end ≥ 2 cm past the tear bar | 1.0 |
| `end_inside` | In the bucket, right way round, lid latched, free end shut inside | 0.5 |
| `lid_open` | In the bucket, right way round, lid not latched | 0.25 |
| `reversed` | In the bucket the wrong way round | 0.0 |
| `other` | Not in the bucket, or nothing done | 0.0 |

"Right way round" is the roll's winding direction: the sign of (paper root − roll centre) × (paper direction) along
the printer's width. It doesn't depend on how far the roll has turned in the bucket, so turning the roll in place
can't fake it.

## Before you submit (in order)

1. `python build_assets.py`, then open both `.usda` files in Isaac Sim and press Play (see the CHECK list at the
   top of `build_assets.py`). Fix the cradle orientation, the lid lip and the strip if they misbehave.
2. Load `task.py` with a zero policy for 200 steps: the printer must stay latched and still; the roll's free end
   must flop down behind it. Write where it lies into `task.json` → setup.
3. Find the YAM finger link names (`taskkit.FINGER_LINK_KEYS`) — not used for scoring here, but the other tasks
   need them too.
4. Run `demo.py` and tune it (see the CHECK list at its top) until `result.json` shows `task_score: 1.0`. If the
   free end won't go through reliably open-loop, record a teleoperated demo instead.
5. Run `--no-turn` and confirm 0. Also try by hand (or a modified script): roll right way round + lid not pressed
   shut → 0.25; free end left inside → 0.5.
6. Run Dream-1 at 2,400 steps; put both scores into `scores` (e.g. `Dream-1 run: 0.0; demo: 1.0`).
7. Encode the videos (`ffmpeg -i in.mp4 -c:v libx264 -pix_fmt yuv420p -movflags +faststart out.mp4`), take the
   setup screenshot, and upload: `python ../../tools/upload_media.py . --video dream.mp4 --demo demo.mp4 --screenshot setup.png`.
8. Check the task.json length limits (fidelity ≤ 500 characters, setup and scoring ≤ 4000 each) and that nothing
   in the text, screenshot or videos identifies the team.
9. Pre-review with `notes/judge-prompt.md` (on the main branch's working folder) before opening the PR.

## Runtime, seeds and reset

- Run limit: 2,400 steps (2 minutes at 20 Hz).
- Runtime per trial: TODO (measure; Isaac Sim's start-up is about 25 s once shaders are cached).
- Seeds: fixed layout (`-E object_jitter=0`); no seed needed.
- Reset: see `reset_notes` in task.json. The `before_reset` hook re-latches the lid and straightens the paper.

## Third-party assets and licences

| Asset | Source | Licence |
|---|---|---|
| Printer, roll, sticker and logo | Built from primitives by `build_assets.py` | Same as this repository |
| DejaVu Sans Bold (sticker and logo text) | dejavu-fonts.github.io | Bitstream Vera / DejaVu licence (free to use and redistribute) |
| `scenekit.py`, `demokit.py` | Robocurve program template | As distributed by the program |
