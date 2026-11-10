# Phase 1 checkpoint — stopped for Claude handoff

Manan requested that engineering stop and an appendable Claude handoff be created
on 11 November 2026. Read `CLAUDE_HANDOFF.md` below his original project brief.
No further simulations were started after that request.

User approved Phase 1 and the bounded position-reference governor. Commits,
pushes, merges and Phase 2 remain unapproved. Ask for fresh cursor approval.

- Branch: `feature/manan-simulink-apriltag-yaw-study`, from `e7a231eff2f9239ff0d6948a058cab19b4b5757c`.
- Lecturer revision: `9d80ddab01b20b01a19cb849b5af5424ca0ae07a`.
- Original circle executed in MATLAB R2026a: 15 s, 15001 samples, finite position outputs.
- Original circle: 3D RMSE 0.383743 m; after 5 s 0.064710 m; final error 0.011931 m.
- Original wrapped-yaw reference minimum rate: -627.518531 rad/s. This is a source reference defect.
- Initial source/branch permission failures were resolved through reviewed execution. Ordinary sandbox shell access may still fail; do not alter ACLs or global Git configuration.
- No raw flight logs or official arena drawing have been supplied in the workspace.
- Raw position-step mission executed: 68.85 s, 68851 finite samples, completed.
- That run is a negative diagnostic: max speed 8.8174 m/s, max height 1.9142 m
  against 0.5 m, max Euler-yaw error 179.8723 degrees, ground penetration
  0.0430164 m. Centre stayed inside the assumed demo boundary. Return error
  before descent was 0.0356178 m; a tiny final endpoint error is not landing validation.
- Raw data/evidence: `results/fixed_yaw_demo/`, `evidence/fixed_yaw_demo.json`.
- Original wrapper output/evidence exists in `results/original_circle/` and
  `evidence/original_circle.json`. Time-weighted original RMSE is 0.383582 m;
  the earlier 0.383743 m number used sample means.
- Latest governor/config: default `bounded_position`, 0.15 m/s sampled reference
  bounds, 360 s cap, 12-element mission state. No controller/plant retuning.
- Latest 12 pure mission checks passed, including governor bounds/all corners
  and the production rotated-frame transform. Six earlier synthetic analysis
  checks passed. These are not governor plant-execution evidence.
- Governor Simulink execution is PENDING. New quaternion/body-rate result fields
  and light plot exports are also pending an end-to-end run. Old generated SLX
  is a stale raw snapshot; rebuild through `run_baseline`.
- Next: fresh cursor approval if desktop needed, then `run_baseline('demo')`,
  verify actual results, finish Phase 1 documentation, pause before Phase 2.

Keep every source change inside this directory. Preserve lecturer assets byte-for-byte. Do not claim PX4 equivalence, validated touchdown physics or official geometry.
