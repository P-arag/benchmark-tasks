# Replace a laser printer's toner cartridge, taking the new one's protective cover off first

A compact laser printer's door is shut over a used toner cartridge; a new one lies on its packing foam with its
orange protective cover still clipped on; a return box waits to the left. The instruction is "Replace the toner
cartridge with the new one." Doing it right means opening the door, drawing the old cartridge out past its detent
into the return box, taking the cover off the new one (with the cover on, the cartridge jams 12 mm short of home),
pushing the new one in until it clicks, and shutting the door.

> **STATUS: NOT RUN.** Written without the simulator; search for `# CHECK (run)`.

| File | What it is |
|---|---|
| `task.json` | Task details (scores/media to fill) |
| `task.py` | Printer (door + detent), two cartridges, cover, foam blocks, return box; door magnet monitor; `outcome()` etc. |
| `geometry.py` | Dimensions (the cover's 12 mm vs the cavity's 6 mm headroom is what makes the cover matter) |
| `build_assets.py` | Writes `assets/laser_printer.usda`, `cartridge_old.usda`, `cartridge_new.usda`, `cover.usda` |
| `demo.py` | Open-loop demo; `--keep-cover` must score 0 |
| `scenekit.py`, `demokit.py`, `usdkit.py`, `taskkit.py` | Helpers |

## Run a trial

```bash
python build_assets.py
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the toner cartridge with the new one." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/toner/dream
python demo.py task.py demo_out
python demo.py task.py demo_cover --keep-cover --no-record    # expect 0
```

## Scoring

| `outcome()` | Condition | Score |
|---|---|---|
| `done` | New cartridge home + cover off, old one in the return box, door shut | 1.0 |
| `old_not_boxed` | New home + cover off, door shut, old not in the box | 0.5 |
| `door_open` | New home + cover off, door open | 0.25 |
| `other` | New not home, cover on, anything dropped off the table | 0.0 |

The two cartridges are told apart by their labels (USED / NEW), and in the scorer by name.

## Before you submit (in order)

1. Build, open each file in Isaac Sim (CHECK list in `build_assets.py`). Measure the force needed to pull the old
   cartridge past the detent and to push the new one home, then compare them with a real printer (8–10 N is a
   guess).
2. Zero-policy load: the old cartridge must stay home, the cover must stay on the new one, the door must stay shut.
3. Check the cover really blocks seating: in a script, push the new cartridge in with its cover on. It must stop
   short of `SEAT_TOL`.
4. Tune the demo (CHECK list in `demo.py`), confirm `--keep-cover` → 0, run Dream-1, fill `scores`, measure the
   runtime, set the final `run_limit`, upload media, check the length limits, and pre-review with
   `notes/judge-prompt.md`.

## Runtime, seeds and reset

Run limit 3,600 steps (a 3-part task; confirm against the demo's length). Runtime TODO. Fixed layout, no seed.
Reset per `reset_notes`.

## Third-party assets and licences

| Asset | Source | Licence |
|---|---|---|
| Printer, cartridges, cover, box | Built from primitives by `build_assets.py` / task-file boxes | Same as this repository |
| DejaVu Sans Bold (labels) | dejavu-fonts.github.io | Bitstream Vera / DejaVu licence |
| `scenekit.py`, `demokit.py` | Robocurve program template | As distributed by the program |
