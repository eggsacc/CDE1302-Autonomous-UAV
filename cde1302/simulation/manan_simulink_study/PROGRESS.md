# Checkpoint — Phases 1, 2 and 3 complete (12 Nov 2026)

Phase 3 done: Manan ran `run_phase3_yaw` (14 runs, 38.5 min, all completed).
`RESULTS_PHASE3.md` has the results and the final recommendation: keep fixed
yaw (tangent improved nothing; worse dead-reckoned target error in 3/3
pairs; 6.5 % frames gated; sensitive to flow-sensor offset). Two of the
report's pre-run predictions (written by Claude) were wrong and are reported
as such (gate loss 6.5 % not 15-17 %; nose lags turns by ~33 deg). A completion-scoring bug (100 Hz
grid missing the DONE tick) was fixed in `nav_metrics`/`run_nav_scenario`.
Optional, not run: the multi-seed landmark-layout grid `run_nav_experiments`.
UWB dropped. Commit of Phase 2/3 files awaits Manan's approval.

Phase 2 write-up: `RESULTS_PHASE2.md`. Manan runs all Simulink simulations;
Claude does not. GUI runs are exported to the workspace as `guiRuns`.
`run_phase2_checks` passed 15/15 on Manan's desktop (11 Nov 2026, 9.2 min):
touchdown landing ideal 0.002 m, A 0.127 m, C 0.007 m. GUI live view and
guiRuns export confirmed. Next: the multi-seed grid (`run_nav_experiments`).

Updated 11 November 2026 (Singapore time) by Claude. Manan's original brief
remains the authoritative spec. Phase 1 results are below, unchanged.

## Status in one line

Phase 1 is complete and committed (`922f574`). Phase 2 and 3 were merged by
Manan into one milestone: three random target tags, any number of known floor
landmarks placed in a GUI, fixed or tangent yaw, Methods A/B/C. UWB is
dropped. All Phase 2/3 code is written and its logic tests pass. **No
estimator-in-the-loop Simulink run has completed yet.** Manan asked to run the
simulations in the desktop and report findings; Claude does not launch them.

## Git state

- Branch **`manan`**, pushed. Study commits: `9d62e69` (study files,
  committed by Manan) and `922f574` (Phase 1 docs, approved).
- Phase 2/3 files below are **uncommitted**. Ask Manan before committing.
- On other branches the folder shows only ignored `results/` and `generated/`
  (~280 MB). Never `git add .` on another branch while they are present.

## Phase 2/3 status

| Piece | Status | Evidence |
|---|---|---|
| Yaw modes + matched turn pauses (`mission_reference`, `study_config`) | Done | 18/18 `check_mission_reference` (12 original Phase 1 checks still pass) |
| Scenario config, seeded targets/sensor errors (`scenario_config`, `anchor_layout`) | Done | covered by `check_nav_core` pairing test |
| Sensor + estimator core (`nav_core`) | Done | 10/10 `check_nav_core` |
| Simulink wiring (`build_study_model` nav mode, `nav_estimator`) | Compiles | model update passed; full run interrupted at Manan's request |
| Method B port (`nav_postcorrect`) | Done | 4/4 `check_postcorrect` |
| Run/metrics (`run_nav_scenario`, `nav_metrics`, `nav_tag_report`) | Written | needs a Simulink run |
| GUI (`uav_study_gui`) | Written | Code Analyzer clean; never opened yet |
| Experiments (`run_nav_experiments`, `plot_nav_experiments`, `plot_nav_comparison`) | Written | needs Simulink runs |
| Results and yaw recommendation | Not started | depends on Manan's runs |

Logic-test highlights (synthetic truth, not flight evidence):
- Ideal sensors: estimate equals truth exactly while translating and turning.
- Camera footprint at 0.8 m: 1.566 m across x 1.253 m along, 0.08 m ahead.
- 0.05 m/s flow bias, 20 s hover over a landmark: A drifts 1.00 m, C 0.036 m.
- A 0.5 m outlier is rejected; a 0.30 m step error converges without overshoot.
- 3 deg placement heading error falls to 0.06 deg over the home tag in 5 s.
- Method B recovers a 3 deg / 11 cm frame offset exactly; with time-growing
  creep, auto mode picks the time model (target error 0.154 m -> 0.000 m).

Turn plan, team flight (0.8 m, 1.0 m lanes, 0.30 m/s): 11 leg junctions
(-14, +14, eight x 90, -140.6 deg); 31.4 s of matched pauses; planned ~159 s.
Strict tangent yaw would need unbounded yaw rate at each junction.

## First Simulink results (Manan's desktop runs, 11 Nov 2026)

Seed 1, team flight, 5 landmarks (`home_corners5`), fixed yaw. 71-124 s wall
clock per 156 s mission; every mission completed.

| | Ideal sensors | A | B (post-flight, from A) | C (online) |
|---|---:|---:|---:|---:|
| Estimate error RMS / max (airborne) | 0.002 / 0.004 m | 0.109 / 0.232 m | track 0.015 m | 0.062 / 0.179 m |
| Heading error RMS | 0.00 deg | 1.72 deg | - | 0.65 deg |
| Target mean error (3/3 confirmed) | 0.002 m | 0.042 m | 0.006 m | 0.026 m |
| True distance to planned route RMS (sweep) | 0.054 m | 0.119 m | same flight as A | 0.090 m |
| Max tilt | 3.2 deg | 4.2 deg | same flight as A | 11.4 deg |
| Landmark frames in view / used | 43 / 39 % | 48 / 44 % | - | 44 / 39 % (9 rejected) |

Method B chose `hybrid` (leave-one-out 3.0 cm vs global 6.9, time 6.3).
C's tilt spike comes from estimate jumps at landmark corrections.

**Artefact found and fixed:** landing errors of 0.31 / 1.04 / 0.30 m were
measured after touchdown. Below 8 cm flow was switched off and the no-floor
lecturer model let the drone slide ~0.3 m with no horizontal feedback.
Fix: flow cut-off lowered to 2 cm (`flowMinHeightM`), landing now scored at
touchdown (true height <= 2 cm) with the later slide reported separately.
The table above predates the fix for landing only; other rows are unaffected.
Re-score existing runs with `res.metrics = nav_metrics(res)`; the cut-off
change needs new runs.

## What Manan runs next (desktop MATLAB, in this folder)

1. `out = run_phase2_checks();` (~6 min): logic tests, perfect-sensor wiring
   check, realistic A(+B) and C. Paste the VERDICTS and SUMMARY blocks.
   Saves `results/phase2_checks/runs.mat` and `comparison.png`.
2. `uav_study_gui`: one run, confirm the live map moves, then
   `print_nav_summary(guiRuns)` and paste.
3. Only if 1 is all PASS: `T = run_nav_experiments();` (30 runs, ~1 h),
   then `summarise_nav_experiments(T)` (paste) and `plot_nav_experiments()`
   (sends landmarks.png and yaw.png to results/nav_experiments/).

Report: verdict/summary blocks, any red error text word for word (not the
"not connected"/"Demux4" warnings), and the PNGs.

## Known caveats

- The first smoke run was slow (still simulating after ~10 min wall time).
  Two speed fixes were added afterwards and are untested: the mission block
  gets a small `studyMissionConfig`, and the estimator caches its config.
- Batch MATLAB on this machine hangs on any figure creation. Stalled batch
  processes from 04:19 (PIDs 24508, 48584) may still exist; stopping them was
  blocked by the permission classifier. Close them in Task Manager if wanted.
- All Phase 2/3 numbers will rest on assumed sensor errors (`scenario_config`),
  the lecturer RPT controller (not PX4) and a study EKF (not EKF2).

## Phase 1 results (unchanged)

Governor run, 256.25 s, all 14 corners, stage DONE. Max horizontal speed
0.181 m/s, max height 0.561 m (0.5 m commanded), max tilt 3.02 deg, RMS
setpoint error 0.041 m, return error 0.036 m, ground penetration 0.040 m
(no contact model). Raw position-step run for contrast: 8.82 m/s, 1.914 m,
108.9 deg tilt. Evidence: `evidence/fixed_yaw_demo_bounded_position.json`.

## Geometry and data still missing

No official arena drawing, no `log_185_UnknownDate.ulg`, no `log_sample.zip`,
no flown `config_used.yaml`. Demo geometry is an assumption on every figure.

Standing rules: keep every change inside this directory; preserve lecturer
assets byte-for-byte; never claim PX4 equivalence, validated touchdown,
official geometry, or flight validation of simulated sensor effects.
