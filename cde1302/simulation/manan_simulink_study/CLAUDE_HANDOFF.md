# CONTINUATION HANDOFF FOR CLAUDE — append below Manan's original README

> **Superseded for current status (11 Nov 2026, later the same day):** the
> governor run described as pending below has since executed and completed,
> and the study now lives on branch `manan`. See `PROGRESS.md` for the live
> checkpoint. This file is kept unchanged below as history.

**Read Manan's original project brief above this section in full. It remains the authoritative specification. Follow its phases, engineering explanations, validation standards, safety restrictions and approval gates exactly. This handoff adds implementation history and current state; it does not replace or relax any requirement.**

Manan asked Codex to stop engineering work and prepare this handoff on **11 November 2026, Singapore time**. Do not interpret the existence of source files as evidence that every feature has run. Resume from the specific checkpoint below, without repeating the entire initial audit or claiming Phase 1 is complete.

## 1. The immediate assignment

Continue **Phase 1 only**: establish a reproducible, lecturer-based, fixed-yaw Simulink baseline with ideal feedback. The original lecturer example has executed successfully. A raw position-step mission also executed, but its behaviour was too aggressive to serve as a credible slow indoor baseline. Manan approved a bounded position-reference governor in response. Its source and pure logic checks are now present, but **the governor has NOT yet been run through the quadrotor Simulink model**.

Your next engineering task is to execute and assess that approved governor version, document its results honestly, and pause for Manan's review before Phase 2. Do not start the GUI, AprilTag estimator, yaw comparison or UWB yet.

## 2. Authorization and interaction rules carried forward

- Manan approved the audit plan, creating `feature/manan-simulink-apriltag-yaw-study` from the pinned `claude-one-shot` commit, and implementing Phase 1.
- After seeing the raw-step results, he explicitly approved the bounded position-reference adaptation. It must retain the lecturer's controller/vehicle and zero velocity/acceleration feedforward.
- He has **not** approved committing, pushing, merging, Phase 2, or unrelated operational-code changes.
- He then explicitly requested that Codex stop and write this handoff. No governor vehicle simulation was started after that request. The already-running pure logic/syntax check completed successfully and was read for handoff accuracy.
- **Ask for approval before taking control of his mouse/cursor again.** His temporary two-minute/five-minute windows have expired; they are not ongoing permission. He specifically said: “ask for approval again if need cursor”. Do not bring apps forward, click or type through computer use until a new approval is given.
- Explain each major step using his required six fields: **What I'm doing / Why / Files involved / What happened / What it means / Next step**. Keep routine logs concise and teach the physical meaning.
- Use Singapore time. Preserve the standing AGENTS.md rules supplied above, including the Google Doc source-of-truth rule if he requests a rules update. No rules synchronization was requested or performed during this project.
- Keep changes inside the dedicated study directory. Preserve all existing untracked study work. Do not run `git clean`, destructive reset commands, or change operational firmware.
- Never flash, arm, take off, change PX4 parameters or operate the physical UAV.

## 3. Workspace and exact Git state

```text
Workspace:
C:\CDE1302_mini_proj

Team checkout:
C:\CDE1302_mini_proj\CDE1302-Autonomous-UAV

Lecturer checkout, preserved:
C:\CDE1302_mini_proj\CDE1302_UAV

Study directory:
C:\CDE1302_mini_proj\CDE1302-Autonomous-UAV\cde1302\simulation\manan_simulink_study
```

Actual branch:

```text
feature/manan-simulink-apriltag-yaw-study
HEAD = e7a231eff2f9239ff0d6948a058cab19b4b5757c
git status --short --branch:
## feature/manan-simulink-apriltag-yaw-study
?? cde1302/simulation/
```

There are **no study commits, no staged changes and no pushes**. All new study sources/evidence remain untracked. The lecturer checkout and operational tracked files were not edited. Do not mistake the absence of a tracked diff for absence of work.

Source pins:

- Team: `https://github.com/eggsacc/CDE1302-Autonomous-UAV`, `claude-one-shot`, `e7a231eff2f9239ff0d6948a058cab19b4b5757c`.
- Lecturer: `https://github.com/linfengfrank/CDE1302_UAV`, `main`, `9d80ddab01b20b01a19cb849b5af5424ca0ae07a`.

## 4. What was actually audited

All eight team branches were inspected, including their latest changes. No additional branches were returned at audit time. Refresh only if new upstream information is needed; do not merge them.

| Branch | Audited HEAD | Relevant finding |
|---|---|---|
| `camera-fps` | `0782811c` | Full-resolution laptop AprilTag detection |
| `claude-one-shot` | `e7a231ef` | Integrated mission and home-tag changes; chosen base |
| `docs/restructure` | `03160974` | UAV subsystem documentation |
| `feature/loc-tags` | `adbcb83a` | Latest height becomes 0.8 m; landmark/report correction |
| `main` | `222d9160` | Earlier template baseline |
| `nikidudu` | `717ca58d` | Early navigation/planner |
| `uwb-testing` | `561cb760` | UWB work and saved PX4 parameter export |
| `yaw_test_lab5` | `dacbfa50` | Earlier yaw/navigation validation |

`feature/loc-tags` and `claude-one-shot` diverge: feature is 12 commits ahead and two behind, sharing `d43aaf2c`. Feature does not simply contain every latest claude change.

**The actually flashed/flown configuration is unresolved.** READMEs, committed sdkconfig and flight-related commit messages conflict. Claude HEAD enables corner-to-corner trajectory waypoint mode, although documentation describes lookahead sweeps as flown. Feature documentation calls feedforward/localisation unflown while later commits refer to flight fixes. Feature firmware/config is 0.8 m while its Python simulator still uses 0.5 m. Mission logging does not record a firmware Git SHA. Neither branch HEAD can be certified as flown without run configuration and build/log provenance.

The existing Python mission simulator directly assigns trajectory positions; it is not a dynamic quadrotor/controller simulator. Feature landmark correction fits a retrospective 2D translation/rigid/similarity transform and explicitly never sends corrected odometry to PX4. Initial alignment and precision-landing setpoint corrections are separate live actions. Genuine online feedback during search would be new work in Phase 2.

## 5. MATLAB/tools and access findings

Verified installation and execution environment:

```text
C:\Program Files\MATLAB\R2026a\bin\matlab.exe
26.1.0.3346908 (R2026a) Update 5
```

Simulink, Aerospace Blockset, Aerospace Toolbox, Robotics System Toolbox and Optimization Toolbox are installed and passed `license('test',...)`. The original model actually executed, which is stronger evidence than licence-existence tests. Optimization is optional minimum-snap work, not a baseline dependency. R2025b has a partial directory but no usable `bin\matlab.exe` was found.

**No MATLAB MCP tool was loaded in this Codex session.** Manan specifically requested an MCP-only `version` check. Available-tool metadata was searched for MATLAB/Simulink/MathWorks and returned no matches. No fallback was used for that specific check. He subsequently instructed Codex to return to the original project using available routes. Do not describe shell execution or Node-based computer use as a MATLAB MCP tool. Claude may have different tools: inspect the actual current inventory rather than assuming.

Runtime observations:

- Restricted `matlab -batch` failed before executing MATLAB code: `System Error: File system inconsistency`.
- Reviewed execution as the normal Windows user ran MATLAB version/licence checks and pure MATLAB logic/syntax checks.
- Full batch Simulink attempts stalled around runtime/file-generation initialization and were stopped. Exact root cause was not established. Do not claim a full batch simulation passed or assume ordinary PowerShell outside Codex has the same issue.
- **The MATLAB desktop is the verified full-simulation route.** The original reproduction and raw mission produced real saved `.mat`, CSV, JSON and PNG outputs there.
- Desktop automation used the computer-use skill and `@oai/sky` through Node REPL, not MATLAB MCP. Future Claude must use its own supported tools/instructions and obtain fresh cursor approval.
- A MATLAB desktop process was left open. Do not assume saved window IDs/process IDs are stable. Other MATLAB processes may belong to Manan's separate MCP investigation; do not terminate them.

Initial sandbox writes failed for both `.git` and study directories. After permissions changed, reviewed execution created the branch/folders. Git then reported different ownership (sandbox account versus Manan). A **command-scoped** trust exception was used for this exact clone, not a global setting:

```powershell
git -c safe.directory=C:/CDE1302_mini_proj/CDE1302-Autonomous-UAV <approved-operation>
```

Do not change ACLs, global Git trust, security settings or sandbox controls to force progress. If access fails, report the actual error and use supported reviewed authorization. Root `apply_patch` worked for files in existing study directories; old child agents retained stale denied access. No child write workaround was used.

## 6. Model architecture and limitations already established

The original RPT model was saved in R2025a. It accepts eleven reference channels:

```text
[x y z vx vy vz ax ay az yaw yawrate]
```

Positions/velocities use local NED, yaw uses radians. Dynamics output 3 is Euler angles in **radians**, confirmed through the SLX wiring. Dynamics output 5 is motor RPM despite a misleading root label.

```text
reference -> RPT position control -> desired thrust/attitude
          -> quaternion attitude control -> body-rate control
          -> mixer -> motors -> rigid-body dynamics
          -> exact position/velocity/quaternion/body-rate feedback
```

There is no sensor estimator, AprilTag camera or EKF in the original loop. Source parameters describe NUSWARM, not the team's calibrated MicoAir vehicle:

- Mass 0.302 kg, arm radius 0.0755 m.
- Inertia diagonal `[3.9195e-4, 4.0515e-4, 6.3890e-4]` kg m².
- Motor time constant 0.02 s; gravity 9.8 m/s²; aerodynamic damping zero.
- Fixed-step `ode4`, 0.001 s.
- RPT XY position/velocity/integral gains approximately 10.9/6.3/3.0; Z 8.92/5.7/2.4.
- Original tilt limit is extremely permissive. Original yaw saturation references the pitch rate-limit value 220 rad/s rather than yaw's 200 rad/s. The source comments label these rad/s; do not invent an intended degrees/s interpretation.

The compiled study sample-time report lists continuous, 0.001 s, 0.05 s, constant and multirate entries. An SLX mixer setting of 0.004 s is not evidence that the entire compiled controller runs at 250 Hz.

The PID model has an eight-channel input; it is not interchangeable with the RPT reference. Minimum-snap assignments contain unfinished placeholders and were not selected as the mission baseline.

There is **no ground-contact, touchdown, motor-disarm or PX4 LAND model**. Do not clamp truth at the floor or call endpoint tracking a validated autonomous landing.

## 7. Source files now present

Everything below is relative to the study directory:

| File | Purpose/current status |
|---|---|
| `README.md` | Project setup and architecture; see this handoff for the latest governor status |
| `PROGRESS.md` | Short resume checkpoint, updated with the stop/handoff state |
| `study_config.m` | Demo/team/assessment presets, explicit frame transform, current governor defaults |
| `mission_reference.m` | 20 Hz Level-2 MATLAB S-function and pure test API |
| `check_mission_reference.m` | Latest 12 logic checks, passed |
| `check_analysis.m` | Six hand-checkable synthetic analysis checks, passed before latest plot styling changes |
| `check_environment.m` | Versions, licence tests, function providers, environment evidence |
| `verify_sources.m` | SHA-256/byte-count checks for five immutable lecturer assets |
| `build_study_model.m` | In-memory RPT copy, mission source/logging/stop wiring, generated inspectable SLX |
| `run_lecturer_example.m` | Original reproduction and evidence/plots |
| `run_baseline.m` | Mission run, signal extraction/alignment, raw outputs, analysis/evidence |
| `analyse_run.m` | Time-weighted errors, stage metrics, boundary/return/endpoint checks, CSV and plots |
| `run_phase1.m` | Analysis fixture, original reproduction, then mission baseline |
| `.gitignore` | Ignores generated models/cache, runtime results, autosaves |
| `vendor/lecturer/PROVENANCE.md` | Attribution and original-source details |
| `vendor/lecturer/.gitattributes` | Disables Git text conversion for immutable vendor bytes |

Five original assets are preserved and verified byte-for-byte: circle script, RPT SLX, `Init_control.m`, `icon/Init.m`, `icon/pixhawk.png`. There is an authored empty-helper-path placeholder for the original `addpath funct` line. Do not edit the five originals or update hashes to hide a discrepancy.

The builder retains the original dynamics/controller and ideal feedback. It changes initialization scope to the copied model workspace, replaces the reference source, logs truth/reference/diagnostics, disables FlightGear/pacing in the **study copy**, and stops at mission completion/failure. Original reproduction retains FlightGear. A batch-only `open_system` -> `load_system` in-memory UI adaptation exists, but full batch execution remains unverified; the saved successful reproduction used no such adaptation.

Two adapter defects were fixed before the successful raw mission:

1. Deleting a reference block left a dangling connected line; explicitly delete its line before reconnecting.
2. Multiplying a diagnostic vector by a zero/one gain to select flags propagated `0 * NaN` into the stop signal, causing immediate stop. Current wiring demultiplexes the actual done/failed flags and ORs them directly. **Do not reintroduce arithmetic flag selection.** Raw output is now saved before postprocessing assertions.

## 8. Actual successful execution evidence

### Original circle — executed

`evidence/original_circle.json` and `results/original_circle/` contain the real reproduction:

| Metric | Result |
|---|---:|
| Duration / samples | 15 s / 15,001 |
| Finite position outputs | Yes |
| Time-weighted 3D RMSE | 0.383582218 m |
| Time-weighted RMSE after 5 s | 0.064704984 m |
| Final 3D position error | 0.011930831 m |
| Minimum reference yaw rate | -627.518531 rad/s |

Earlier console values 0.383743 m / 0.064710 m used sample means rather than time-weighted integration; this explains the small difference. Initial reference is 2 m from initial vehicle position. Wrapped tangent yaw was differentiated directly, producing the huge reference spikes. This is a reference defect, not observed real yaw drift. The circle has no takeoff/return/landing mission.

### Raw position-step fixed-yaw-command mission — executed, physically poor

Preserved legacy output folder: **`results/fixed_yaw_demo/`**.
Compact evidence: **`evidence/fixed_yaw_demo.json`**.

| Metric | Actual result |
|---|---:|
| Duration / samples | 68.85 s / 68,851 |
| Mission state machine completed | Yes; final stage 8 |
| Finite state/reference and correct initial position | Yes |
| Commanded yaw / v,a feedforward | Fixed zero / all zero |
| XY setpoint tracking RMS | 0.916212278 m |
| Sweep active-leg distance RMS | 0.284100460 m |
| Maximum horizontal speed | 8.817394952 m/s |
| Maximum height against 0.5 m command | 1.914215745 m |
| Euler-yaw RMS / maximum error | 30.0704° / 179.8723° |
| Minimum true-centre outer-boundary margin | 0.069891263 m |
| Return error before descent | 0.035617817 m |
| Return settle timeout | No; stable return gate satisfied |
| Final horizontal endpoint error | 0.000729943 m |
| Maximum ground-plane penetration | 0.043016403 m |

An independent calculation from the CSV matched the XY RMS and found maximum tilt about **108.9249°**, with minimum horizontal nose projection **0.03759**. Large Euler-yaw excursions therefore occur alongside extreme attitude/near-vertical orientation. Do not label this independent yaw-induced drift or mistake Euler representation effects for a calibrated real-flight phenomenon.

This run is a useful negative diagnostic, **not a healthy fixed-heading slow-flight baseline**. Its tiny final error does not rescue its speed/altitude/attitude behaviour. Centre boundary compliance is not rotor-clearance compliance. Ground-plane penetration is a failed geometric check and an expected limitation of a free-flight model without contact.

Runtime files include a ~20.9 MB raw SimulationOutput MAT, ~7.2 MB structured run MAT and ~29 MB CSV; these are simulation outputs, not flight logs, and are ignored by Git. Three PNGs were inspected. They initially inherited MATLAB's dark axes with light text on white export margins. An explicit light-export helper has now been written but **those old plots have not been regenerated with it yet**.

### Tests

- Five lecturer asset hashes passed.
- Six synthetic analysis checks passed: known 0.1 m offset/route distance, 1 m integrated travel, full-turn yaw wrap, unavailable return data and incomplete mission handling. These are labelled synthetic, not flight/plant evidence.
- Latest `evidence/mission_logic_checks.json`: **12 checks passed**, including the new governor bounds, visiting every planned corner, timeout handling and the production route transform with a 90° rotation/displaced origin.
- The last already-started reviewed batch check exited 0 with `SOURCE_AND_MISSION_CHECKS_OK`. This is pure logic/source checking, not a governor Simulink run.

## 9. Precisely what changed after the raw run, and remains unexecuted

Manan approved moving the position reference gradually rather than stepping it metres at a time. Current defaults:

```matlab
cfg.mission.guidanceMode = 'bounded_position';
cfg.mission.maxReferenceSpeedMps = 0.15;
cfg.mission.maxVerticalReferenceSpeedMps = 0.15;
cfg.mission.maxTimeS = 360;
```

The same corner order and controller are retained. Reference horizontal displacement is norm-bounded per 0.05 s update; vertical displacement is separately bounded. All velocity, acceleration and yaw-rate feedforward fields stay zero. In bounded mode, corner handoff also waits for the commanded point to reach the corner, preserving the planned vertices. The old `waypoints` mode remains available.

The S-function DWork state grew from nine to **twelve** elements, adding commanded position. `Outputs` computes without mutating state; `Update` commits. Same-time repeated evaluations must not advance twice. This is a documented study reference governor, **not exact flown firmware parity or a PX4 speed-limit implementation**. Command-reference speed limits do not guarantee the same bound on actual UAV speed.

New output names preserve the legacy raw run:

```text
results/fixed_yaw_demo_bounded_position/
evidence/fixed_yaw_demo_bounded_position.json
```

**Neither exists as a successful governor vehicle run yet.** The generated `generated/MananStudy_RPT.slx` is an older raw-run snapshot with stale stored configuration. Do not run it directly against the changed S-function and claim it represents the new defaults. Rebuild using `run_baseline`.

Other newly written, not yet end-to-end rerun changes:

- `run_baseline` retains quaternion/body-rate fields in the structured result.
- `analyse_run` reports tilt/nose-projection context and uses an explicit light export helper.
- `run_phase1` includes the analysis fixture.
- Nonzero frame overrides now flow through actual route construction; the latest pure test exercises that production path.

A useful remaining reporting improvement is to retain native reference timestamps/values and evaluate reference slew at the **20 Hz command times**. Do not differentiate zero-order-held references on the 1 ms truth grid and claim the resulting impulses violate a 0.15 m/s sampled governor bound.

## 10. Exact next steps for Claude

1. Read the original brief above, this handoff, `PROGRESS.md`, source and evidence. Verify branch/status and preserve untracked work. Do not redo Phase 0 wholesale.
2. Check actual available MATLAB tools. If MATLAB MCP is present, use its real supported capabilities. If absent, say so. The previous MCP-only version request was a one-off check, not a prohibition on the subsequently authorized original project routes.
3. Before any computer-use input, ask Manan for fresh cursor approval. Prepare the command first so the handoff is short. Do not automate a terminal through the UI or use an indirect UI shell command.
4. Use the verified MATLAB desktop route, with the study directory on the path, to run the approved governor:

```matlab
cd('C:\CDE1302_mini_proj\CDE1302-Autonomous-UAV\cde1302\simulation\manan_simulink_study');
addpath(pwd);
try
    result = run_baseline('demo');
catch phaseError
    disp(getReport(phaseError,'extended','hyperlinks','off'));
end
```

5. Verify actual governor results: multiple increasing timestamps, finite state/reference, command slew at native ticks, zero feedforward, fixed **commanded and achieved** yaw, height/speed/tilt, visited corners, completion before timeout, return settling, centre boundary margin and unclamped endpoint/ground-plane behaviour. Preserve failures; do not tune the controller or hide bad data without a justified reviewed change.
6. Confirm the logged reference is the actual controller input, truth stays separate, and `estimatedPosition` is empty / `estimateAvailable=false`. There is no sensor estimator in Phase 1.
7. If useful, rerender the preserved raw run using the updated analysis helper. Label that as postprocessing of the same run, not a newly executed flight/simulation. Update compact evidence consistently if adding derived metrics; do not overwrite the original execution timestamp with a fictitious new run time.
8. The exact pinned-team placeholder scenario is available but has not been executed through the plant. If claiming it works, execute it. For raw mode: `cfg=study_config('team_head_placeholder'); cfg.mission.guidanceMode='waypoints'; result=run_baseline(cfg);`. Unknown official boundaries must remain unavailable.
9. Inspect exported plots for clipping, theme contrast, labels and units. Finish README/PROGRESS/results summary with an explicit executed/unexecuted table and actual numerical evidence. Credit lecturer, teammates and Codex accurately.
10. Present the Phase 1 educational summary and pause for approval before Phase 2. Ask separately before committing/pushing, with status/diff summary/intended message/destination. Do not declare the whole project complete.

Reproduce the original circle again only if a relevant change/failure warrants it. Its saved execution evidence already exists. `run_lecturer_example` uses the base workspace and `close all`; use a fresh session or protect unrelated user work. Do not close/save unrelated models.

## 11. Geometry and datasets still missing

No official arena drawing, `log_185_UnknownDate.ulg`, `log_sample.zip` or flown `config_used.yaml` was made available to the study. Accessible project/lab/backup/data locations were searched; the Windows user folder was inaccessible to the restricted search. This was not an exhaustive proof that the files do not exist on the machine. Ask for accessible paths when needed. Do not assume the two recordings match.

Default demo geometry is explicitly assumed:

```text
Outer x 0..5.4, y 0..4.5 m
Home arena (0.6, 0.6), local NED (0,0,0)
Start square x/y 0.15..1.05 m
Search x 1.2..4.8, y 0.45..4.05 m
Lane spacing 0.6 m; not a calibrated camera swath
Arena-to-local yaw 0 rad; not geographic North/East alignment
```

Only the outer dimensions, 3.6 m search width and 0.9 m start-square size came from Manan's official-diagram description. Search length/offsets/start placement remain assumptions. `study_config('assessment')` deliberately errors until authoritative geometry is available. `study_config('demo',frameOverrides)` supports explicit local origin/arena heading before route construction.

The pinned-team placeholder uses x 0..4.8, y -1.35..1.0 and 2.5 m lanes. Feature uses a different 3.68 x 3.22 m grid-based rectangle. Neither establishes the official assessment layout. Both team configs expect six targets; the original requested future study default is **three unknown targets**, separate from anchors.

`log150_real_path.png` was inspected and is a derived “likely real path” reconstruction, not independent ground truth and not log185. A saved UWB-branch PX4 1.15.4 parameter export is not proof of log185 settings.

## 12. Future-phase facts to preserve from the audit

- ESP32 AprilTag camera and optical-flow/range sensor are separate systems.
- Camera config assumes 90° down and 0.08 m forward offset, with that offset explicitly marked for measurement. Older helpers assume a different mount.
- Hardcoded 320x240 intrinsics and 120 mm tag16h5 size exist; calibration/distortion/mount uncertainty is unverified.
- Current projection is a level-flight yaw-only 2D approximation. Future full body/camera/world rotations must be explicit.
- A 30°/s yaw-rate detection rejection gate supplies a code-backed availability effect. Do not inject a yaw-position bias and call it independent evidence.
- `tags_on_floor:false` is not a wall-normal/occlusion/visibility model. A downward camera cannot measure an anchor it cannot see.
- Source MAVLink loop is 20 Hz and laptop telemetry 10 Hz; camera/flow rates/delays from settings are not measured flight timing.
- A single anchor can constrain translation under assumptions; arbitrary scale/rotation need more geometry. Five corner/centre anchors are not automatically optimal.
- Post-flight map correction cannot improve an already flown trajectory. Online corrected position must actually replace controller feedback, and mission guidance must not retain hidden truth when claiming online navigation effects.
- Fixed/tangent yaw comparisons must share scenarios/reference/sensor conditions, handle wrapping/zero speed/rate limits, and report infeasible corner yaw demands quantitatively.
- UWB remains optional and deferred until the core works.

**Claude: follow Manan's complete original README above exactly. Continue the approved Phase 1 governor validation from this checkpoint; do not silently skip requirements, repeat the initial audit unnecessarily, mislabel tests as simulations, or move to later phases without the required review.**
