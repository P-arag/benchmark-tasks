# Clip a lead's bolt snap onto a dog collar's swinging D-ring

A plush dog form wears a collar with a free-swinging D-ring; a short lead lies on the table with its bolt snap. The
instruction is "Clip the lead onto the dog's collar." The robot has to hold the snap upright in one hand and pull
the trigger back with the other, get the D-ring's wire through the 9 mm gap without pushing the ring away, then let
the bolt close.

> **STATUS: NOT RUN.** Written without the simulator; search for `# CHECK (run)`. The scorer's linking test is
> plain math and was checked on synthetic loops (linked → ±1, apart → 0, open gate → 0).

| File | What it is |
|---|---|
| `task.json` | Task details (scores/media to fill) |
| `task.py` | Dog form + D-ring, lead + snap + bolt; monitor computes the linking number every step |
| `geometry.py` | Dimensions, hook/D-ring polygons, `linking_number()` (exact segment formula) |
| `build_assets.py` | Writes `assets/dog_collar.usda`, `assets/lead.usda` |
| `demo.py` | Open-loop demo; `--no-trigger` must score 0 |
| `scenekit.py`, `demokit.py`, `usdkit.py`, `taskkit.py` | Helpers |

## Run a trial

```bash
python build_assets.py
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clip the lead onto the dog's collar." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/lead/dream
python demo.py task.py demo_out
python demo.py task.py demo_notrigger --no-trigger --no-record   # expect 0
```

## Scoring

| `outcome()` | Condition | Score |
|---|---|---|
| `clipped` | Hook + shut bolt linked with the D-ring; no finger within 3 cm; snap still 1 s | 1.0 |
| `held` | Linked and shut, still in hand | 0.5 |
| `gate_open` | Ring wire inside the hook, bolt not shut | 0.25 |
| `other` | Anything else | 0.0 |

## Before you submit (in order)

1. Build; check the D-ring's swing and the bolt's gap (CHECK list in `build_assets.py`).
2. Find the YAM finger link names (`taskkit.FINGER_LINK_KEYS`). `released` depends on them. With no matches it is
   always False, and the task can never reach 1.0.
3. Sanity-check the monitor: write the snap's pose so the ring is in the hook (from a script) and print
   `_STATE['linked']`.
4. Tune the demo (CHECK list in `demo.py`). A teleoperated demo is a reasonable fallback, since this is fiddly
   open loop.
5. Confirm `--no-trigger` → 0, run Dream-1, fill `scores`, upload media, check the length limits, and pre-review with
   `notes/judge-prompt.md`. Expect the reviewer to look at the dog form. Its shape doesn't affect the mechanics,
   but if the asset library has a licensed dog mesh, use it for the visuals.

## Runtime, seeds and reset

Run limit 2,400 steps. Runtime TODO. Fixed layout, no seed. Reset per `reset_notes`.

## Third-party assets and licences

| Asset | Source | Licence |
|---|---|---|
| Dog form, collar, D-ring, lead, snap | Built from primitives by `build_assets.py` | Same as this repository |
| `scenekit.py`, `demokit.py` | Robocurve program template | As distributed by the program |
| Linking-number formula | Klenin & Langowski, Biopolymers 54 (2000), reimplemented | (method, not code) |
