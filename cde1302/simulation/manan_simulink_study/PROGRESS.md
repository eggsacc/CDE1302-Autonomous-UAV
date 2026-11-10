# Phase 1 checkpoint — governor executed, awaiting Manan's Phase 2 approval

Updated 11 November 2026 (Singapore time) by Claude, continuing from Codex's
`CLAUDE_HANDOFF.md`. Manan's original brief remains the authoritative spec.

## Status in one line

The approved bounded position-reference governor has now **executed through the
lecturer's quadrotor Simulink model** and completed the full mission. Phase 1
deliverables exist; Phase 2 (targets, anchors, estimator, GUI) is **not started**
and needs Manan's approval.

## Git state

- Working branch: **`manan`**. Manan committed the study himself as `9d62e69`
  ("commit", 04:36) and pushed it to `origin/manan`. The branch was cut from
  `feature/manan-simulink-apriltag-yaw-study` at `e7a231e`, which is still the
  pinned `claude-one-shot` base.
- The study exists only on `manan`. On other branches the folder shows only the
  ignored `results/` and `generated/` (~280 MB), which then appear untracked.
  Never `git add .` on another branch while those folders are present.
- These documentation updates were made after `9d62e69` and are uncommitted.
  Claude does not commit or push without Manan's explicit approval.

## Executed / not executed

| Item | Status | Evidence |
|---|---|---|
| Lecturer circle reproduction (R2026a) | Executed (by Codex, desktop) | `evidence/original_circle.json`, `results/original_circle/` |
| Raw position-step mission (`waypoints` mode) | Executed; negative diagnostic | `evidence/fixed_yaw_demo.json`, `results/fixed_yaw_demo/` |
| **Bounded governor mission (`bounded_position`)** | **Executed, completed, stage DONE** | `evidence/fixed_yaw_demo_bounded_position.json`, `results/fixed_yaw_demo_bounded_position/` |
| Lecturer asset hashes (5 files) | Passed before the governor run | console log |
| Mission logic checks (12) | Passed before the governor run | `evidence/mission_logic_checks.json` |
| Native 20 Hz command-slew check | Passed on the governor run (max 0.150 m/s both axes) | one-off check from `raw_simulation.mat`, numbers below |
| Replay of the governor run (`animate_run`) | Viewed by Manan in the desktop | — |
| `team_head_placeholder` preset through the plant | **Not executed** | — |
| Official arena geometry | **Unresolved**; demo geometry is assumed | `study_config.m` |
| PX4 logs `log_185`, `log_sample.zip` | **Not available** in workspace | — |
| Ground contact / landing physics | **Not modelled** (by design) | floor penetration visible in results |

## Governor run — actual numbers

Simulated 04:05:51, post-processed 04:28:55 (Singapore), MATLAB R2026a Update 5,
fixed-step ode4 at 1 ms, 256 251 samples, 256.25 s. Stage times: takeoff 0 s,
hover 3.25 s, sweep 8.05 s, return 204.9 s, settle 240.15 s, descend 244.9 s,
endpoint hold 249.9 s, done 256.25 s. All 14 sweep corners commanded in order.

| Metric | Raw step run | Governor run |
|---|---:|---:|
| Max horizontal speed | 8.817 m/s | 0.181 m/s |
| Mean sweep speed | 0.896 m/s | 0.153 m/s |
| Max height (0.5 m commanded) | 1.914 m | 0.561 m |
| Max tilt | 108.9° | 3.02° |
| Max yaw error | 179.87° | 0.023° |
| RMS setpoint error XY | 0.916 m | 0.041 m |
| Max setpoint error XY | 5.411 m | 0.142 m |
| RMS distance to active leg | 0.284 m | 0.025 m |
| Closest approach, long-lane corners | — | ≤ 0.0001 m |
| Closest approach, short cross-lane corners | — | 0.030 m |
| Return error before descent | 0.036 m | 0.036 m |
| Final horizontal endpoint error | 0.0007 m | 0.0008 m |
| Min centre margin to outer boundary | 0.070 m | 0.398 m |
| Max ground-plane penetration | 0.043 m | 0.040 m |

All integrity, reference, boundary and mission checks pass. The single false
check is `noGroundPlanePenetration`, which is the expected free-flight model
limitation, not a controller fault. There is still no tracking pass threshold;
these are measurements, not a pass/fail verdict.

Physical reading: with the command walked at 0.15 m/s, the lecturer's RPT loop
trails its moving setpoint by about 4 cm on straight lanes, cuts the short
cross-lane corners by about 3 cm, and overshoots takeoff to 0.56 m before
settling. These follow from ideal feedback and the NUSWARM parameters; they are
not evidence about the team's MicoAir aircraft or PX4.

## How it was run (and the batch-mode caveat)

- `run_baseline('demo')` ran to completion in **ordinary PowerShell batch mode**
  (`matlab -batch`), unlike Codex's sandbox. Simulation, raw save, metrics and
  CSV all completed.
- The **PNG export hangs in batch mode** (figure creation with 256k-point lines
  never returns; CPU idles). The batch process was stopped after about 20 min.
  Manan then ran `postprocess_baseline('demo')` in the MATLAB desktop, which
  rebuilt everything from the saved `raw_simulation.mat` in about a minute.
  Desktop is the verified route for plots; batch is fine for simulation.
- Benign warnings seen: unconnected spare outputs of the study's status demux,
  a `Demux4` warning that belongs to the lecturer's model, and "Diagnostic
  Viewer is not ready" (batch mode has no viewer).

## Files added or changed by Claude

| File | Change |
|---|---|
| `postprocess_baseline.m` | New. Builds result/metrics/plots/run.mat/evidence from `raw_simulation.mat` or directly from `sim()` output. Records simulation and post-processing timestamps separately. Keeps native 20 Hz reference samples. |
| `run_baseline.m` | Now calls `postprocess_baseline` after saving raw output; no behaviour change to the simulation itself. |
| `animate_run.m` | New. Replays a saved run (top-down + 3D, airframe cross, command marker, stage label). Replay of logged data, not a live simulation. |
| `analyse_run.m` | Cosmetic: tiled-layout titles exported dark, stage tick labels literal. Metrics unchanged. |
| `README.md`, `PROGRESS.md`, `CLAUDE_HANDOFF.md` | Status brought up to date; handoff body kept as history. |

The current PNGs were exported before the cosmetic fix; rerun
`postprocess_baseline('demo')` in the desktop to refresh them.

## Team context noted for Phase 2

`feature/loc-tags` gained two teammate commits on 11 Nov (`5f799ae`, `690637e`):
in-flight drift correction for the tag report in `cde1302/mission/localisation.py`
and a camera bench tool `cde1302/mission/tools/cam_bench.py`. They do not touch
this study. Review them before designing Methods B and C in Phase 2.

## Geometry and data still missing

Unchanged from the handoff: no official arena drawing, no `log_185_UnknownDate.ulg`,
no `log_sample.zip`, no flown `config_used.yaml`. Demo geometry (outer 5.4 × 4.5 m,
search x 1.2..4.8 / y 0.45..4.05, start square 0.15..1.05, home (0.6, 0.6), 0.6 m
lanes) is an **assumption** labelled on every figure.

## Next step

Phase 1 review by Manan. If approved, Phase 2: three random target tags, known
anchors with camera field-of-view visibility, a simple documented estimator with
methods A/B/C, and the arena GUI. Ask separately before any commit or push.

Standing rules: keep every change inside this directory; preserve lecturer assets
byte-for-byte; never claim PX4 equivalence, validated touchdown or official geometry.
