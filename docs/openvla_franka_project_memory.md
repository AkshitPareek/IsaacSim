# OpenVLA Franka Pick-Place Project Memory

## Goal

Build toward a Franka Panda pick-and-place system in Isaac Sim where a vision-language-action model can:

- Pick up a box placed at varied positions.
- Place it at varied language-specified targets.
- Move from observer/calibration mode to bounded closed-loop autonomy step by step.

## Current Stable Baseline

- Isaac Sim is running with Vulkan disabled in app source configs because the local Windows/NVIDIA stack crashes on the Vulkan renderer path.
- The scripted Franka pick-place controller is restored as the stable control baseline.
- `vla_pick_place.py` runs an observer-only calibration experiment:
  - Isaac scripted controller still executes the task.
  - Overhead camera frames are captured.
  - OpenVLA is queried over HTTP.
  - Scripted goals, robot state, cube state, target state, images, VLA outputs, and VLA errors are logged.

## Current Important Files

- `source/standalone_examples/api/isaacsim.robot.manipulators/franka/pick_place.py`
- `source/extensions/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/franka/pick_place/pick_place.py`
- `source/standalone_examples/api/isaacsim.robot.manipulators/franka/vla_pick_place.py`
- `source/standalone_examples/api/isaacsim.robot.manipulators/franka/pick_place_d3d12.py`
- `source/standalone_examples/api/isaacsim.robot.manipulators/franka/vla_pick_place_d3d12.py`
- `source/apps/isaacsim.exp.base.python.kit`
- `source/apps/isaacsim.exp.base.python.d3d12.kit`

## Swarm Operating Rules

- Keep user Vulkan/D3D12 app config changes intact.
- Do not modify the built-in scripted controller unless a task explicitly requires it.
- Prefer adding capability to the observer/calibration path first.
- Implementers should work on mutually exclusive files or clearly separated sections.
- Each implementer should produce a short plan before coding, then implement, test, and report changed files.
- Generated calibration data belongs outside commits unless explicitly requested.

## Initial Task Split

- [x] Lead: refine task list, review implementer plans, keep this memory file current.
- [x] Implementer A: add randomized cube and target placement to observer runs.
- [x] Implementer B: add dataset inspection/report tooling for calibration CSV outputs.
- [x] Implementer C: harden OpenVLA query/logging controls and runtime diagnostics without changing robot control.

## Milestone Checklist

- [x] Restore stable scripted Franka pick-place baseline.
- [x] Keep OpenVLA out of robot control and move it to observer/calibration mode.
- [x] Disable Vulkan in local Isaac app configs so Windows launches reliably on this machine.
- [x] Add D3D12 wrapper examples for explicit renderer testing.
- [x] Add observer CSV/image logging.
- [x] Add randomized cube and target positions for dataset coverage.
- [x] Add dataset analyzer for calibration CSVs.
- [x] Add VLA query diagnostics, dry-run controls, latency logging, and 7D validation.
- [x] Generate a larger randomized observer dataset with real OpenVLA responses.
- [x] Add dataset quality gates so bad calibration runs are obvious.
- [x] Add language-conditioned target variants and target labels/colors.
- [x] Design a dry-run action adapter that maps VLA/action signals to bounded Franka waypoints.
- [x] Validate the adapter offline against logged scripted goals.
- [ ] Enable model control for Phase 0 reach only.
- [ ] Expand to language-conditioned place waypoint selection.
- [ ] Consider full closed-loop policy control only after waypoint phases are stable.

## Swarm Round 2 Task Split

- [x] Lead: turn the next milestone into concrete branch tasks and review implementer plans.
- [x] Implementer D: add dataset quality gates/report thresholds to the analyzer.
- [x] Implementer E: add target variants, labels, and language templates in observer mode.
- [x] Implementer F: design and implement an offline dry-run action adapter report without robot control.

## Completed Swarm Round 2 So Far

Merged branches:

- `codex/openvla-quality-gates`
- `codex/openvla-target-variants`
- `codex/openvla-action-adapter-report`

New or updated capabilities:

- `analyze_vla_calibration.py` can enforce optional quality gates and exits nonzero when requested gates fail.
- `vla_pick_place.py` logs per-run `target_label` and `instruction`.
- `vla_pick_place.py` supports target labels and instruction templates while keeping robot control scripted.
- `report_vla_action_adapter.py` evaluates a bounded VLA xyz-to-waypoint adapter offline.

Test evidence:

- Quality gates pass syntax checks and fail as expected for impossible thresholds such as `--min-runs 999`.
- Target variants completed 3 dry-run scripted runs with `green target`, `red target`, and `blue target`.
- Offline adapter report runs on the existing CSV, but flags Phase 0 as `LOW SAMPLE` with only 2 usable rows.

Current next step:

- Generate a larger randomized observer dataset with real OpenVLA responses.
- Run quality gates and the adapter report on that fresh dataset.
- Use the resulting evidence to decide whether Phase 0 model control is ready for a guarded live experiment.

## Completed Swarm Round 3

Merged branches:

- `codex/openvla-balanced-collection`
- `codex/openvla-phase-quality-gates`
- `codex/openvla-adapter-diagnostics`
- `codex/openvla-run-summary`

New or updated capabilities:

- `vla_pick_place.py` can cycle or balance target labels across runs:
  - `--target-label-mode random|cycle|balanced`
  - `--runs-per-label`
- `analyze_vla_calibration.py` reports query successes by phase, target label distribution, latency percentiles, empty camera frames, and run-level coverage.
- `analyze_vla_calibration.py` can gate required phase sample counts and label diversity.
- `report_vla_action_adapter.py` reports per-phase adapter behavior, axis sign agreement, simple correlations, and grid-search scale diagnostics.

Real-small dataset summary:

- Dataset: `vla_calib_logs_real_small/calibration.csv`
- Runs: `2`
- Rows: `560`
- VLA success: `27 / 28`
- Mean latency: `3505.11 ms`
- Labels sampled: only `green target`
- Query successes by phase: phase `0=5`, `1=4`, `4=8`
- Run-level placement success-like count: `2 / 2` under `0.08 m` XY final distance.
- Adapter report marks phases `0`, `1`, and `4` as `LOW SAMPLE`.
- Phase 0 shows `x` sign disagreement in this tiny dataset, so live model control is not ready.

Recommended next real collection:

```powershell
cd C:\Users\Akshit\Projects\isaacsim\_build\windows-x86_64\release
$env:OPENVLA_LOG_DIR="C:\Users\Akshit\Projects\isaacsim\vla_calib_logs_balanced_real"
$env:OPENVLA_SERVER_URL="http://localhost:8000/act"
$env:OPENVLA_QUERY_EVERY="10"
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --device cuda --ik-method damped-least-squares --runs 15 --seed 2027 --target-labels "red target,green target,blue target" --target-label-mode balanced --runs-per-label 5 --instruction-template "pick up the blue cube and place it on the {target_label}" --openvla-timeout 60
```

Recommended validation:

```powershell
cd C:\Users\Akshit\Projects\isaacsim
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\analyze_vla_calibration.py --csv vla_calib_logs_balanced_real\calibration.csv --required-phases 0,1,4 --min-query-successes-per-phase 20 --min-label-count 3 --min-runs-per-label 5 --max-empty-camera-frames 2 --max-vla-latency-p95-ms 10000
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\report_vla_action_adapter.py --csv vla_calib_logs_balanced_real\calibration.csv --per-phase --diagnose-axes --grid-search-scales 0.05,0.1,0.25,0.5,1.0
```

Balanced real dataset result:

- Dataset: `vla_calib_logs_balanced_real/calibration.csv`
- Runs: `15`
- Rows: `4200`
- VLA success: `419 / 420`
- Labels: `5` runs each for `red target`, `green target`, and `blue target`
- Query successes by phase: phase `0=89`, `1=60`, `4=120`
- Latency p95: `2813.13 ms`
- Run-level placement success-like count: `15 / 15` under `0.08 m` XY final distance.
- Quality gates passed for required phases, label balance, latency, and empty camera frames.

Offline adapter validation result:

- Phase 0: raw VLA xyz adapter worsened mean distance by `8.63%`.
- Phase 1: raw VLA xyz adapter worsened mean distance by `13.18%`.
- Phase 4: raw VLA xyz adapter improved mean distance by `0.90%`.
- Phase 0 axis diagnostics show poor sign agreement: `x=11.24%`, `y=29.21%`, `z=47.19%`.
- Conclusion: the dataset is good; the naive raw delta adapter is not ready for Phase 0 live control.

Current next step:

- Build stronger offline calibration tools before live model control.
- Try phase-specific affine/linear adapters and axis/sign mappings against the balanced dataset.
- Only enable Phase 0 live control if an offline adapter beats the scripted baseline with enough margin and stable axis diagnostics.

## Completed Swarm Round 4 Local Pass

New or updated capabilities:

- `fit_vla_action_adapter.py` fits an offline affine adapter per phase:
  - `desired_delta_axis = b + wx*vla_dx + wy*vla_dy + wz*vla_dz`
  - train/test split is by `run_id`
  - no Isaac imports and no robot control
- `fit_vla_action_adapter.py` can write a JSON config with per-phase coefficients and `ready_for_control` metadata:
  - `--write-config path\to\affine_adapter_config.json`
  - `--min-test-improvement 10`

Affine adapter result on `vla_calib_logs_balanced_real/calibration.csv`:

- Holdout `0.3`: Phase 0 test worsened by `5.00%`, Phase 1 improved by `27.29%`, Phase 4 improved by `25.19%`.
- Holdout `0.2`: Phase 0 improved by only `0.44%`, Phase 1 improved by `24.15%`, Phase 4 improved by `32.91%`.
- Conclusion: Phase 1 and Phase 4 have a real offline adapter signal, but Phase 0 is still not ready for live model control.

Recommended next step:

- Add a guarded dry-run/live-control scaffold for Phase 1 or Phase 4 first, not Phase 0.
- Keep Phase 0 scripted until we build a better object-local or waypoint predictor.
- If the goal remains Phase 0 autonomy first, collect richer visual/action labels or switch the model target from raw deltas to object/waypoint prediction.

Useful config export command:

```powershell
cd C:\Users\Akshit\Projects\isaacsim
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\fit_vla_action_adapter.py --csv vla_calib_logs_balanced_real\calibration.csv --phase 0,1,4 --holdout-fraction 0.3 --min-test-improvement 10 --write-config vla_calib_logs_balanced_real\affine_adapter_config.json
```

## Completed Swarm Round 5 Local Pass

New or updated capabilities:

- `vla_pick_place.py` can load an affine adapter config in observer-only mode:
  - `--adapter-config` / `OPENVLA_ADAPTER_CONFIG`
  - `--adapter-dry-run` / `OPENVLA_ADAPTER_DRY_RUN`
  - `--adapter-max-delta` / `OPENVLA_ADAPTER_MAX_DELTA`
- The observer CSV now records adapter dry-run fields:
  - readiness, acceptance, rejection reason
  - proposed goal, proposed delta, delta norm
  - distance from proposed adapter goal to scripted goal
- `fit_vla_action_adapter.py` now writes numeric phase IDs in exported JSON configs, while the live loader remains backward-compatible with string phase IDs.

Dry-run evidence:

- One full D3D12 scripted run completed in `vla_calib_logs_adapter_dryrun_test` after the adapter loader patch.
- A follow-up run in `vla_calib_logs_adapter_dryrun_test2` populated adapter fields before an Isaac/PhysX shutdown crash.
- Phase 1 dry-run sample looked plausible:
  - adapter error to scripted goal: `0.0136 m`
  - adapter delta norm: about `0.0313 m`
- Phase 4 dry-run sample looked unsafe despite the offline held-out score:
  - adapter error to scripted goal: `1.5277 m`
  - adapter delta norm: about `0.4038 m`
  - likely caused by the affine phase 4 fit amplifying a repeated OpenVLA xyz action.

Conclusion:

- Do not enable affine adapter live control yet, including Phase 4.
- Keep using the scripted controller for motion while logging adapter dry-run proposals.
- Next safe step is to collect more dry-run adapter rows and gate by live dry-run behavior, not only offline split metrics.
- For live autonomy, prefer a bounded waypoint/object-local predictor or a much stricter adapter with clamps and phase-specific validation.

## Swarm Round 6 Task List

Milestone: adapter dry-run reporting and gates before any live VLA control.

Current safety decision:

- No live VLA or affine adapter control is approved for any phase.
- Scripted Franka control remains the only motion authority.
- Adapter output may be computed, logged, and reported only in dry-run mode.
- A future live-control proposal must pass explicit dry-run gates first and require a separate lead review before code enables robot control.

Implementer tasks:

- Implementer G: add adapter dry-run quality gates to the reporting/analyzer path.
  - Gate adapter rows by phase, accepted/rejected counts, rejection reasons, finite numeric fields, max proposed delta norm, and max/mean adapter error to scripted goals.
  - Acceptance criteria: a command can fail nonzero when adapter dry-run behavior exceeds configured limits; it prints a concise pass/fail summary by phase; it works without Isaac imports; it can parse existing Round 5 dry-run CSVs.
- Implementer H: improve adapter dry-run report readability and evidence export.
  - Add per-phase summaries for readiness, acceptance rate, rejection reasons, error-to-scripted-goal percentiles, delta-norm percentiles, and worst rows with run/step/phase identifiers.
  - Acceptance criteria: one report gives enough evidence for lead review without opening the CSV manually; unsafe Phase 4-style proposals are obvious; output is stable enough to paste into this memory file.
- Implementer I: run a larger scripted adapter dry-run collection with the current affine config and no live control.
  - Collect balanced target-label runs with adapter dry-run enabled, scripted controller unchanged, fresh log directory, and real OpenVLA responses when available.
  - Acceptance criteria: at least 15 completed scripted runs or a documented simulator failure with partial logs; target labels remain balanced; adapter CSV fields are populated; no generated data is committed unless explicitly requested.
- Implementer J: propose stricter adapter clamps and phase allowlist defaults, but keep them dry-run only.
  - Review existing adapter config/load behavior and define conservative defaults for accepted phases, max delta, finite-value checks, and rejection logging.
  - Acceptance criteria: proposal or implementation cannot enable live control by default; rejected adapter proposals are logged with clear reasons; any live-control flag remains absent or hard-disabled unless separately approved.

Round 6 acceptance gates before any live VLA-control discussion:

- Dataset gates pass for coverage, label balance, VLA query success, empty camera frames, and latency.
- Adapter dry-run gates pass on a fresh balanced dataset for any candidate phase.
- Candidate phase has enough accepted adapter dry-run samples, with low error to scripted goal and no large outlier proposals.
- Rejection reasons are explainable and bounded; no NaN/Inf or malformed adapter fields appear.
- Phase 0 remains scripted unless a new offline and dry-run method shows a clear, stable improvement over scripted-goal matching.
- Phase 4 remains blocked until dry-run reports show the large-error proposal pattern is eliminated.

## Completed Swarm Round 6 So Far

New or updated capabilities:

- `analyze_vla_adapter_dryrun.py` analyzes adapter dry-run CSV logs without Isaac imports.
- It supports older and newer adapter dry-run schemas:
  - recomputes `adapter_delta_norm` if the explicit column is missing
  - recomputes `adapter_error_to_scripted` if goal fields are present
  - treats blank numeric fields as missing, not zero
  - treats queried VLA rows as successful only when all 7 action fields are present and `vla_error` is blank
- It reports per-phase:
  - adapter readiness and acceptance rate
  - adapter sample counts
  - delta norm and error-to-scripted percentiles/max
  - rejection reasons and VLA errors
  - worst adapter rows by error and by delta norm
- It can fail nonzero on dry-run gates:
  - `--required-phases`
  - `--min-adapter-samples-per-phase`
  - `--min-accepted-rate`
  - `--max-adapter-error-p95`
  - `--max-adapter-error-max`
  - `--max-adapter-delta-p95`
  - `--max-adapter-delta-max`

Verification on `vla_calib_logs_adapter_dryrun_test2/calibration.csv`:

- Parsed `161` rows across phases `0,1,2,3,4`.
- Phase 1 dry-run sample remained plausible: delta norm `0.0312914 m`, error `0.0135648 m`.
- Phase 4 dry-run sample failed hard: delta norm `0.404124 m`, error `1.5277 m`.
- Gate command with `--required-phases 1,4 --min-adapter-samples-per-phase 1 --max-adapter-delta-max 0.08 --max-adapter-error-max 0.1` failed as expected.
- Conclusion remains unchanged: no affine adapter live control; collect a larger scripted dry-run dataset and gate it with this analyzer.

Fresh scripted adapter dry-run collection:

- Dataset: `vla_calib_logs_adapter_dryrun_next/calibration.csv`
- Command used the D3D12 Isaac Python launcher, scripted Franka control, real OpenVLA HTTP, adapter dry-run enabled, and `OPENVLA_ADAPTER_MAX_DELTA=0.05`.
- Runs: `6`, rows: `1680`, target labels balanced at `2` runs each for red/green/blue.
- VLA queries: `168`, successes: `167`, errors: `1` first-frame empty camera frame.
- VLA latency p95: `2585.94 ms`.
- Query successes by phase: phase `0=35`, `1=24`, `4=48`.
- Dataset quality gates passed for required phases, label balance, empty camera frames, and latency.
- Scripted run-level success-like count was `5 / 6`; run `004` missed final target distance with `0.528938 m`, so future collection gates should also require scripted final placement quality.

Adapter dry-run result on the fresh collection:

- Phase 1:
  - samples: `24`
  - accepted: `22 / 24`
  - delta norm max: `0.0505235 m`
  - error-to-scripted max: `0.0600677 m`
  - looks plausible but still needs more data and a slightly consistent threshold policy.
- Phase 4:
  - samples: `48`
  - accepted: `0 / 48`
  - delta norm p95/max: `0.570568 / 0.586031 m`
  - error-to-scripted p95/max: `0.603796 / 0.702003 m`
  - failed gates with `--max-adapter-delta-max 0.08 --max-adapter-error-max 0.1`.
- Conclusion is stronger now: the affine adapter must not control Phase 4. Phase 1 is the only current candidate for further dry-run investigation, and Phase 0 remains scripted.

## Swarm Round 7 Task List

Milestone: decide whether Phase 1 deserves a dry-run-only promotion path, while rejecting and redesigning the Phase 4 affine adapter.

Current safety decision:

- No live VLA or affine adapter control is approved for any phase.
- Scripted Franka control remains the only motion authority.
- Round 7 may promote only Phase 1 dry-run evidence quality, not robot authority.
- Phase 4 affine adapter output is rejected for control consideration due to repeated large deltas, high error-to-scripted goals, and zero accepted samples in the fresh dry-run collection.
- Any future Phase 4 work must be treated as adapter redesign, not threshold loosening.

Implementer tasks:

- Implementer K: define Phase 1-only dry-run promotion gates in `analyze_vla_adapter_dryrun.py` or its docs.
  - Acceptance criteria: gates can be run for phase `1` alone; require enough samples, high accepted rate, bounded p95/max delta norm, bounded p95/max error-to-scripted, no NaN/Inf fields, and no unexplained rejection spikes; the command must fail if Phase 4 is accidentally included as a candidate.
- Implementer L: collect a larger Phase 1-focused scripted dry-run dataset with balanced targets and current adapter clamps.
  - Acceptance criteria: scripted control only; at least `15` completed runs or documented simulator failure with partial logs; target labels balanced; scripted final placement quality passes; Phase 1 adapter fields are populated; generated logs are not committed unless explicitly requested.
- Implementer M: prepare a concise Phase 1 promotion report.
  - Acceptance criteria: report includes dataset quality gates, Phase 1 adapter dry-run gates, accepted/rejected counts, rejection reasons, delta/error percentiles, worst rows, and a clear recommendation of `continue dry-run`, `redesign`, or `request separate live-control review`.
- Implementer N: reject and redesign Phase 4 adapter approach.
  - Acceptance criteria: document why the current affine Phase 4 adapter is blocked; propose a replacement design such as object-local waypoint prediction, phase-specific bounded target selection, or a non-affine/clamped model; include offline and dry-run validation gates before any future Phase 4 control discussion.

Round 7 acceptance gates:

- Phase 1 dry-run promotion requires fresh dataset gates to pass for coverage, label balance, VLA success, latency, empty camera frames, and scripted final placement quality.
- Phase 1 adapter gates must pass with conservative max-delta and error-to-scripted thresholds, stable acceptance rate, finite numeric fields, and no large outlier proposals.
- Promotion means only that Phase 1 remains a candidate for more dry-run evidence or a separate lead-reviewed live-control proposal; it does not enable live control.
- Phase 0 remains scripted.
- Phase 4 current affine adapter remains rejected even if Phase 1 passes; Phase 4 needs a redesigned adapter and new offline plus dry-run evidence.
- Any code path that could give VLA/adapter live robot authority remains absent or hard-disabled unless separately approved.

Round 7 data diagnosis:

- Phase 1 looks plausible because it is a local descent problem:
  - mean delta norm `0.0434 m`
  - p95 delta norm `0.0505 m`
  - accepted `22 / 24`
  - mean error-to-scripted `0.0299 m`
  - typical deltas are small and mostly vertical, around `(+0.002, -0.001, -0.043)`.
- Phase 4 fails because the affine adapter emits transport-sized global deltas while runtime treats them as local `ee_pos + adapter_delta` goals:
  - accepted `0 / 48`
  - mean delta norm `0.5040 m`
  - p95 delta norm `0.5706 m`
  - max delta norm `0.5860 m`
  - max error-to-scripted `0.7020 m`
- Phase 4 failure is not caused by target-label imbalance or VLA availability:
  - dry-run dataset was balanced across red, green, and blue targets
  - Phase 4 VLA success was `48 / 48`
- Recommendation: do not extend this affine adapter to Phase 4. Use Phase 1 only as a dry-run scaffold/milestone. The next real autonomy direction should be an object-local adapter that conditions on EE, cube, target, and phase semantics, and outputs bounded local servo steps rather than one-shot global transport deltas.

## Swarm Round 8 Task List

Milestone: validate a larger Phase 1-only scripted dry-run dataset before any live-control discussion.

Current safety decision:

- No live VLA or affine adapter control is approved in Round 8.
- Scripted Franka control remains the only motion authority.
- Round 8 scope is Phase 1 dry-run evidence only; Phase 0 stays scripted and Phase 4 is not a candidate.
- Phase 4 affine remains rejected due to repeated transport-sized deltas, high error-to-scripted goals, and zero accepted samples in the fresh dry-run evidence.

Implementer tasks:

- Implementer O: collect a larger Phase 1-only adapter dry-run dataset with real OpenVLA responses when available.
  - Acceptance criteria: scripted control only; Phase 1 adapter logging enabled; fresh log directory; balanced target labels; at least `15` completed runs or a documented simulator failure with partial logs; generated logs are not committed unless explicitly requested.
- Implementer P: run dataset quality and Phase 1 adapter gates on the new collection.
  - Acceptance criteria: required Phase 1 sample count passes; VLA success coverage, latency, empty-camera, target-balance, scripted placement, finite numeric fields, acceptance rate, delta-norm p95/max, error-to-scripted p95/max, and worst-row checks are reported.
- Implementer Q: compare the larger Phase 1-only dry-run evidence against Round 7 results.
  - Acceptance criteria: report shows accepted/rejected counts, rejection reasons, percentiles, worst rows, and whether Phase 1 behavior is stable across target labels and runs.
- Implementer R: prepare a lead decision note.
  - Acceptance criteria: recommendation is one of `continue Phase 1 dry-run`, `tighten gates and recollect`, or `redesign adapter`; it must not request live control unless a separate future review is explicitly opened after Round 8 evidence is accepted.

Round 8 collection acceptance gates:

- Pass: Phase 1-only dry-run collection is balanced, reproducible, finite, low-latency enough for logging, and shows stable bounded local proposals with no large outliers.
- Fail: any live-control path is enabled, Phase 4 is included as a candidate, target labels are imbalanced, VLA/adapter fields are sparse, scripted placement quality regresses, or Phase 1 produces repeated large deltas/errors.
- Safety rule: failure keeps the project in observer-only scripted control; success only permits a later lead-reviewed discussion, not live control.

## Swarm Round 9 Task List

Milestone: complete a strict 15-run Phase 1 adapter dry-run validation with final placement gates before any live-control proposal.

Current safety decision:

- No live VLA or adapter control is approved in Round 9.
- Scripted Franka control remains the only motion authority for all phases.
- Phase 1 is the only adapter candidate under review, and only in dry-run logging mode.
- Phase 0 remains scripted. Phase 4 affine remains rejected and must not be included as a candidate.
- Generated dry-run logs stay out of commits unless explicitly requested.

Implementer tasks:

- Implementer S: collect the Round 9 15-run Phase 1 dry-run dataset.
  - Acceptance criteria: use scripted control only; real OpenVLA responses when available; fresh log directory; exactly `15` completed runs unless simulator failure is documented; balanced target labels with `5` runs each for red/green/blue; Phase 1 adapter fields populated; no generated logs committed.
- Implementer T: run baseline dataset gates with strict final placement checks.
  - Acceptance criteria: required phases and label balance pass; VLA success coverage is adequate; latency and empty-camera gates pass; every completed run has final cube-to-target XY distance at or below `0.08 m`; fail the round if any run exceeds the final placement gate.
- Implementer U: run Phase 1 adapter dry-run gates.
  - Acceptance criteria: phase `1` only; enough accepted samples across all labels; finite adapter numeric fields; high accepted rate; bounded delta norm p95/max; bounded error-to-scripted p95/max; worst rows are reported with run/step/label; command fails if phases `0` or `4` are accidentally treated as candidates.
- Implementer V: prepare the Round 9 lead decision note.
  - Acceptance criteria: summarize collection command, dataset gates, final placement gate results, Phase 1 adapter gates, target-label stability, rejection reasons, worst rows, and recommendation: `continue dry-run`, `tighten and recollect`, or `open separate live-control review`.

Round 9 strict gates:

- `15 / 15` scripted runs must complete, or the simulator failure must be documented with partial logs and no promotion decision.
- Target labels must be balanced at `5` runs each for red, green, and blue.
- Final placement quality is mandatory: every completed run must end within `0.08 m` XY cube-to-target distance. One miss blocks promotion.
- Phase 1 adapter proposals must stay bounded, finite, and locally plausible with no repeated large outliers.
- Phase 4 must remain blocked regardless of Phase 1 results.
- Passing Round 9 permits only a separate lead-reviewed live-control discussion; it does not enable live control.

## Swarm Round 10 Task List

Milestone: make Round 9/10 collection safer by ensuring only Phase 1 adapter dry-run is considered or logged as a candidate while scripted control remains sole authority.

Current safety decision:

- No live VLA or adapter control is approved in Round 10.
- Scripted Franka control remains the only motion authority for every phase.
- Only Phase 1 adapter dry-run rows may be labeled, counted, or summarized as candidate evidence.
- Phase 0 stays scripted, and Phase 4 affine remains blocked for candidate status.
- Phase 4 `ready_for_control` metadata in the old affine adapter config is stale offline metadata and must not be promoted into Round 9/10 candidate logs, reports, gates, or decisions.

Implementer tasks:

- Implementer W: audit Round 9/10 collection commands and notes for candidate scoping.
  - Acceptance criteria: collection remains scripted-control only; adapter dry-run is enabled only for logging; Phase 1 is the sole candidate phase; Phase 0 and Phase 4 are never described as candidate phases.
- Implementer X: verify analyzer/report commands cannot accidentally promote Phase 4.
  - Acceptance criteria: gates are run with phase `1` only; any summary or decision note treats Phase 4 as blocked even if old config metadata says ready; candidate counts exclude Phase 4 rows.
- Implementer Y: prepare a short Round 10 safety note.
  - Acceptance criteria: note confirms scripted control authority, Phase 1-only dry-run candidate logging, no generated logs committed, and no promotion of old Phase 4 ready metadata.

Round 10 strict gates:

- Pass: Round 9/10 evidence names only Phase 1 dry-run as candidate and clearly separates dry-run proposals from scripted control.
- Fail: any log/report/decision promotes Phase 4, treats old `ready_for_control` metadata as current approval, or implies VLA/adapter has robot authority.
- Safety rule: passing Round 10 improves documentation and review hygiene only; it does not enable live control.

## Completed Swarm Round 10

Collection:

- Command: scripted-control Isaac Sim collection with real OpenVLA queries, affine adapter dry-run logging, and `--adapter-enabled-phases 1`.
- Log directory: `vla_calib_logs_phase1_adapter_dryrun_round10`.
- Scope: `15` runs, balanced target labels, `5` runs each for red/green/blue.
- Safety: scripted Franka remained the only motion authority; OpenVLA and adapter outputs were observer-only.

Dataset gate result:

- Overall: PASS.
- Rows/runs: `4200` rows, `15 / 15` runs.
- VLA: `419 / 420` successes, `1` empty camera frame, latency p95 `2807.48 ms`.
- Labels: red/green/blue balanced at `5` runs each.
- Final scripted placement: `15 / 15` success-like runs, final distance mean `0.0307576 m`, max `0.0669118 m`, under the strict `0.08 m` gate.

Phase 1 adapter dry-run gate result:

- Overall: PASS.
- Phase 1 samples: `60`.
- VLA successes: `60 / 60`.
- Adapter ready: `60 / 60`.
- Adapter accepted: `55 / 60` (`0.917`).
- Delta norm: mean `0.0438247 m`, p95 `0.0505235 m`, max `0.0506179 m`.
- Error to scripted: mean `0.0306145 m`, p95 `0.0592297 m`, max `0.061037 m`.
- Rejections: `5` tiny cap misses just above `0.05 m`; no large Phase 1 outliers.

Phase 4 safety check:

- Intentional Phase 4 candidate gate: FAIL, as desired.
- Phase 4 VLA observations existed (`120 / 120` successes), but adapter candidate samples were disabled by the Phase 1 allowlist.
- Phase 4 adapter-ready rate was `0 / 120`, with rejection reason `phase disabled by adapter phase allowlist`.

Round 10 decision:

- Phase 1 now has a clean larger dry-run evidence set.
- Phase 4 remains blocked.
- Passing Round 10 does not enable live control; next work should be a separate lead-reviewed Phase 1-only live-control design gate or another dry-run focused on replacing scripted Phase 1 with a bounded controller shadow plan.

## Isaac Runtime Speed-Improvement Task List

Purpose:

- Reduce wall-clock time for scripted-control OpenVLA/adapter evidence collection without weakening safety gates.
- Keep scripted Franka as the only motion authority unless a separate live-control review explicitly approves otherwise.
- Prefer runtime flags, launch presets, and report hygiene before changing simulation behavior.

Safety constraints:

- Do not touch generated log directories when tuning runtime presets.
- Do not relax final placement, label-balance, VLA-success, empty-camera, latency, or adapter dry-run gates just to make a run appear faster.
- Any speed preset that changes physics/rendering fidelity must be validated against a known-good baseline before it becomes the default.
- Phase 1 remains the only dry-run adapter candidate; Phase 0 stays scripted and Phase 4 remains blocked.

Task list:

- [ ] Measure a baseline wall-clock profile for the current Round 10 command: total runtime, startup time, per-run duration, query count, latency p95, final placement quality, and adapter gate result.
- [ ] Add a lightweight timing summary to the collection/report workflow if the existing console and CSV outputs do not already make runtime bottlenecks obvious.
- [ ] Confirm the minimum camera/image settings needed for OpenVLA queries and adapter dry-run evidence; avoid saving extra images unless a debugging run requires them.
- [ ] Compare `OPENVLA_QUERY_EVERY` values against Phase 1 sample-count gates so fast runs still collect enough accepted adapter rows.
- [ ] Keep target labels balanced in every preset; use `--target-label-mode balanced` and set `--runs-per-label` explicitly.
- [ ] Test whether shorter runs or fewer total runs still satisfy required Phase 1 samples; do not promote a preset unless its gates pass on fresh data.
- [ ] Document any renderer or launcher choice that improves stability or speed on this Windows/NVIDIA setup, with D3D12 remaining the known explicit fallback path.
- [ ] Add a run-summary note after each collection that records preset name, seed, run count, query cadence, elapsed time, and pass/fail gate status.
- [ ] If simulator startup dominates runtime, consider batching multiple presets in one launched Isaac session only if logs remain cleanly separated and no generated logs are committed.
- [ ] If OpenVLA latency dominates runtime, test server-side batching/caching only in observer mode and only if CSV rows still reflect the exact query response used for each frame.

Proposed collection presets:

| Preset | Purpose | Runs | Labels | Query cadence | Adapter scope | Expected use |
| --- | --- | ---: | --- | --- | --- | --- |
| Fast smoke | Verify launch, logging schema, OpenVLA connectivity, and Phase 1 adapter dry-run plumbing quickly. | `3` | `1` each red/green/blue | `OPENVLA_QUERY_EVERY=20` | `--adapter-enabled-phases 1` | Use before longer collections or after small code/runbook changes. Not enough for promotion. |
| Medium evidence | Catch most runtime and adapter regressions while keeping collection time moderate. | `9` | `3` each red/green/blue | `OPENVLA_QUERY_EVERY=10` | `--adapter-enabled-phases 1` | Use for candidate preset validation and regression checks before spending time on the full run. |
| Full gate | Reproduce the strict Round 10 decision-quality evidence. | `15` | `5` each red/green/blue | `OPENVLA_QUERY_EVERY=10` | `--adapter-enabled-phases 1` | Required before any lead-reviewed promotion discussion; must pass dataset, final-placement, and Phase 1 adapter gates. |

Preset commands:

```powershell
# Fast smoke
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --device cuda --ik-method damped-least-squares --runs 3 --seed 2041 --target-labels "red target,green target,blue target" --target-label-mode balanced --runs-per-label 1 --instruction-template "pick up the blue cube and place it on the {target_label}" --adapter-dry-run 1 --adapter-max-delta 0.05 --adapter-enabled-phases 1
```

```powershell
# Medium evidence
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --device cuda --ik-method damped-least-squares --runs 9 --seed 2042 --target-labels "red target,green target,blue target" --target-label-mode balanced --runs-per-label 3 --instruction-template "pick up the blue cube and place it on the {target_label}" --adapter-dry-run 1 --adapter-max-delta 0.05 --adapter-enabled-phases 1
```

```powershell
# Full gate
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --device cuda --ik-method damped-least-squares --runs 15 --seed 2043 --target-labels "red target,green target,blue target" --target-label-mode balanced --runs-per-label 5 --instruction-template "pick up the blue cube and place it on the {target_label}" --adapter-dry-run 1 --adapter-max-delta 0.05 --adapter-enabled-phases 1
```

Preset interpretation:

- Fast smoke passes only if the app launches, the CSV is populated, all labels appear once, OpenVLA connectivity works, Phase 1 adapter fields appear, and scripted control is unchanged.
- Medium evidence passes only if label balance, VLA query coverage, final scripted placement, finite adapter fields, and Phase 1 dry-run bounds look consistent with Round 10.
- Full gate passes only if it satisfies the strict Round 9/10 gates: `15 / 15` completed scripted runs, `5` runs per label, final XY distance at or below `0.08 m` for every run, and Phase 1 adapter p95/max delta and error remain bounded.
- No preset enables live control; passing a preset only determines whether the next collection or review step is worth running.

## Completed Isaac Runtime Speed Pass 1

Implemented opt-in runtime knobs:

- `--headless` / `ISAACSIM_HEADLESS` / `OPENVLA_HEADLESS`.
- `--camera-resolution` / `OPENVLA_CAMERA_RESOLUTION`.
- `--openvla-enabled-phases` / `OPENVLA_ENABLED_PHASES`.
- `--openvla-save-image-every` / `OPENVLA_SAVE_IMAGE_EVERY`.
- `--openvla-max-image-saves` / `OPENVLA_MAX_IMAGE_SAVES`.
- Runtime summary counters for sampled frames, HTTP queries, VLA errors, and saved images.
- Startup estimated sample/HTTP counts now respect the VLA phase allowlist.

Verification:

- `python -m py_compile source\standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py` passed.
- `vla_pick_place.py --help` exposes the new runtime options.
- No-HTTP headless smoke completed `1` scripted run in about `42.5 s` with `--openvla-enabled 0`, `--openvla-save-images 0`, and `--camera-resolution 160 160`.
- Real OpenVLA Phase-1-only smoke completed `3` scripted runs in about `121 s` with `--openvla-enabled-phases 1`, `--openvla-save-images 0`, and `--headless 1`.

Real Phase-1-only smoke result:

- Rows/runs: `840` rows, `3` runs.
- HTTP queries: `12`, matching the new estimate; Round 10 would have made `84` queries for the same `3` runs at all phases.
- VLA: `12 / 12` successes, latency p95 `3047.47 ms`, `0` empty camera frames.
- Phase 1 adapter dry-run: PASS, `12 / 12` samples ready and accepted, delta p95/max `0.0499995 m`, error p95/max `0.0591783 m`.
- Phase 4 safety check: no Phase 4 VLA/adapter candidate samples because VLA querying and adapter dry-run were both Phase 1 scoped.
- Strict final placement gate: FAIL for this smoke only, `2 / 3` success-like runs with one final distance `0.0993701 m`. Treat this as a runtime/plumbing smoke, not promotion evidence.

Speed decision:

- Use Phase-1-only VLA querying for Phase 1 adapter work. It reduces full-gate OpenVLA calls from about `420` to about `60`, saving roughly `86%` of model wait time while preserving the exact Phase 1 evidence we need.
- Keep all-phase VLA querying only when collecting data for future phases such as Phase 4 redesign.
- Keep `--openvla-save-images 0` for speed/regression runs unless debugging image quality.
- Keep strict full-gate placement requirements for promotion evidence; do not use fast smoke placement results to approve control changes.

## Swarm Round 11 Task List

Milestone: create a Phase 1-only live-control design gate after Round 10 and Speed Pass 1.

Current safety decision:

- Full live control is not approved.
- No code path may give OpenVLA, the affine adapter, or any learned policy robot motion authority in Round 11.
- Scripted Franka control remains the only approved motion authority while Round 11 designs, reviews, and documents a future Phase 1-only live-control gate.
- Phase 1 is the only phase eligible for design-gate discussion because Round 10 produced bounded dry-run evidence and Speed Pass 1 showed Phase-1-only querying can preserve evidence while reducing query load.
- Phase 0 remains scripted. Phase 4 remains blocked. Old adapter `ready_for_control` metadata must not be treated as current approval.
- Generated logs, screenshots, and calibration datasets stay out of commits unless explicitly requested.

Safety constraints:

- The design gate must be Phase 1-only: query scope, adapter scope, candidate summaries, commands, and review language must name only phase `1`.
- The design gate must not relax Round 10 gates for speed: `15 / 15` scripted runs, balanced labels, final XY distance at or below `0.08 m` for every run, adequate VLA success coverage, bounded latency, no empty-camera spike, and bounded Phase 1 adapter deltas/errors.
- Any future live-control design must be single-step or shadowed Phase 1 waypoint replacement only; it must not approve full pick-place, Phase 0 reach, Phase 4 place, gripper autonomy, continuous closed-loop control, or policy authority across phases.
- Motion limits in the future design must be stricter than the dry-run evidence: bounded Cartesian delta, bounded workspace, finite numeric checks, rate/step limits, and explicit rejection on stale, missing, malformed, or out-of-phase VLA/adapter data.
- The scripted controller must remain available as immediate fallback and must be the default behavior on every launch.
- The design must include an operator-visible abort path before any later implementation can be reviewed.
- Any later implementation proposal must require a separate lead review after this design gate passes; Round 11 itself does not implement robot control.

Implementation tasks:

- [ ] Implementer AA: write the Phase 1-only live-control design specification in this memory file or a linked docs-only section.
  - Acceptance criteria: describes the exact allowed experiment boundary, candidate phase `1`, forbidden phases `0` and `4`, dry-run evidence used, non-goals, and explicit statement that full live control is not approved.
- [ ] Implementer AB: define the pre-live evidence checklist using existing analyzers.
  - Acceptance criteria: checklist names the Round 10 full-gate requirements, Speed Pass 1 Phase-1-only query scope, final placement gate, adapter dry-run gate, required command options, and pass/fail interpretation.
- [ ] Implementer AC: define the future Phase 1 control envelope without coding it.
  - Acceptance criteria: specifies max Cartesian delta, workspace bounds, finite-value checks, phase allowlist, stale-response timeout, one-step command budget, gripper lockout, and default fallback to scripted control.
- [ ] Implementer AD: define abort, fallback, and rollback rules.
  - Acceptance criteria: covers operator abort, adapter rejection, OpenVLA timeout/error, camera/image failure, phase mismatch, simulator instability, final-placement miss, unexpected gripper or joint behavior, and any threshold breach.
- [ ] Implementer AE: prepare the lead review packet template.
  - Acceptance criteria: template requires collection command, git status, changed files, analyzer outputs, worst adapter rows, screenshots only if needed, risk assessment, explicit go/no-go recommendation, and confirmation that no live-control implementation was merged.

Required gates before a later Phase 1 live-control implementation may even be proposed:

- Documentation gate: Round 11 design checklist is complete, reviewed, and still states that full live control is not approved.
- Scope gate: every command and report uses `--openvla-enabled-phases 1` and `--adapter-enabled-phases 1`; no candidate summary includes Phase 0 or Phase 4.
- Dataset gate: a fresh full-gate scripted run passes `15 / 15` completed runs, balanced red/green/blue labels at `5` each, adequate VLA success coverage, latency and empty-camera thresholds, and no generated logs committed.
- Final-placement gate: every completed scripted run ends with cube-to-target XY distance at or below `0.08 m`; one miss blocks promotion.
- Adapter dry-run gate: Phase 1 has enough samples, `100%` VLA success for queried Phase 1 rows, adapter readiness populated, accepted rate consistent with Round 10 or better, p95/max delta and p95/max error bounded, finite fields only, and no repeated outlier pattern.
- Safety-envelope gate: proposed future control limits are tighter than observed Round 10 dry-run maxima or explicitly justified by a lead-reviewed margin.
- Review gate: the lead signs off on a separate implementation task after reading the review packet; passing gates does not itself authorize code changes.

Abort and fallback rules for any later live-control proposal:

- Abort immediately if OpenVLA times out, returns a malformed/non-7D action, returns NaN/Inf, or produces data for a phase other than `1`.
- Abort immediately if camera frames are empty/stale, target labels are missing or imbalanced in the evidence run, or the simulator reports instability.
- Reject the adapter proposal and fall back to scripted Phase 1 if the proposed delta exceeds the configured cap, leaves the workspace envelope, has stale timestamps, or lacks a matching scripted-control context row.
- Fall back to scripted control on any operator stop, keyboard interrupt, unexpected gripper command, unexpected joint jump, phase transition mismatch, final-placement miss, or analyzer gate failure.
- After any abort/fallback event, do not resume live authority in the same run; finish scripted-only if safe, preserve logs, and require a new review before retrying.
- If any ambiguity appears between speed settings and safety evidence, choose the slower known-good full-gate path.

Round 11 pass/fail checklist:

- [x] PASS: docs clearly say full live control is not approved.
- [x] PASS: Phase 1 is the only candidate phase in design language, commands, gates, and review templates.
- [x] PASS: Phase 0 and Phase 4 are explicitly blocked from live-control consideration.
- [x] PASS: all safety constraints, required gates, and abort/fallback rules are documented before implementation work begins.
- [x] PASS: future implementation work is separated into a later lead-reviewed task and defaults to scripted control.
- [ ] FAIL: any code implements robot control, continuous policy control, gripper authority, Phase 0/Phase 4 authority, or a live-control default.
- [ ] FAIL: any gate is weakened for speed, any generated log data is committed unintentionally, or old adapter metadata is used as approval.

## Round 11 Design Spec (AA–AE)

This section satisfies the AA–AE acceptance criteria above with concrete values, drawing on the Round 10 dry-run evidence and the Phase 1 scaffold currently committed off-by-default in `vla_pick_place.py`.

### AA — Phase 1-only experiment boundary

In-scope (one and only one allowed experiment shape):

- Phase: `1` only (transport / approach to grasp). The scripted controller continues to drive Phase 0 (initial reach), Phase 2/3 (descent/grasp), and Phase 4 (place).
- Authority: zero or one Cartesian end-effector pose update per Phase 1 step, derived from `apply_phase1_adapter_control` in `vla_pick_place.py`. No joint control, no gripper authority, no continuous closed-loop control across phases.
- Mode progression: dry-run → shadow (adapter computes, scripted moves) → `live-once` (one full pick-place with Phase 1 adapter pose updates, operator armed, headed display visible).
- Dataset of record: a fresh `15 / 15` Phase 1-only scripted dry-run run that satisfies Round 10 strict gates with the Speed Pass 1 query scope.

Forbidden under Round 11:

- Phase 0 live control. Phase 0 stays scripted.
- Phase 4 live control. The current affine adapter is rejected for Phase 4 due to transport-sized deltas (max delta `0.586 m`, max error `0.702 m`, accepted `0 / 48` in the dry-run evidence).
- Gripper or joint commands from VLA/adapter output.
- Continuous closed-loop control or multi-phase policy authority.
- Default-on live control for any future flag.

Dry-run evidence used (from `vla_calib_logs_phase1_adapter_dryrun_round10`):

- `15 / 15` scripted runs, balanced red/green/blue at `5 / 5 / 5`.
- Phase 1 samples `60`, accepted `55 / 60` = `0.917`, `60 / 60` VLA success.
- Delta norm: mean `0.0438 m`, p95 `0.0505 m`, max `0.0506 m`.
- Error to scripted goal: mean `0.0306 m`, p95 `0.0592 m`, max `0.0610 m`.
- Final placement: mean `0.0308 m`, max `0.0669 m` (under the `0.08 m` gate).
- Phase 4 candidate samples: `0` (allowlist disabled), Phase 4 adapter ready rate `0 / 120`.

Non-goals:

- Improving model accuracy.
- Replacing the scripted controller anywhere outside Phase 1.
- Removing the dry-run/shadow modes once live mode is reviewed.
- Reusing this scaffold for any other phase, robot, task, or model without a new design gate.

Approval state: full live control is **not** approved by Round 11. The scaffold in code is gated off; passing Round 11 only allows a separately reviewed proposal to schedule a `live-once` smoke after the dry-run gates below pass on fresh data.

### AB — Pre-live evidence checklist

Run in this order; every item must pass before lead review can be requested.

1. **Source-control hygiene**: working tree clean for committed code; no `vla_calib_logs_*` directories staged. Verify with `git status` before each evidence collection.
2. **Fast smoke** (`3` runs, `query_every=20`, dry-run only) using the Phase 1 scaffold runbook command for `vla_calib_logs_phase1_scaffold_smoke_dryrun`. Required: `analyze_vla_calibration.py` PASS with `--min-runs 3 --min-label-count 3 --min-runs-per-label 1 --min-query-successes-per-phase 12`; `analyze_vla_adapter_dryrun.py` PASS with `--phase 1 --min-adapter-samples-per-phase 12 --require-ready --min-accepted-rate 0.90 --min-vla-success-rate 1.0 --max-adapter-delta-max 0.055 --max-adapter-error-max 0.07`.
3. **Medium evidence** (`9` runs, `query_every=10`, dry-run only) using the medium runbook command. Required: dataset gate PASS with `--min-runs 9 --min-runs-per-label 3 --min-query-successes-per-phase 36 --max-final-distance 0.08`; adapter gate PASS at the same thresholds as full-gate but with `--min-adapter-samples-per-phase 36`.
4. **Full dry-run gate** (`15` runs, `query_every=10`, dry-run only). Required: every Round 10 numeric threshold reproduced or beaten — `15 / 15` runs, balanced labels at `5` each, final distance max `≤ 0.08 m`, latency p95 `≤ 4000 ms`, empty camera frames `≤ 1`, Phase 1 accepted rate `≥ 0.90`, p95/max delta `≤ 0.052 / 0.055 m`, p95/max error `≤ 0.065 / 0.07 m`.
5. **Phase 4 negative check**: rerun adapter analyzer against the same CSV with `--phase 4`. Must continue to FAIL or report `0` candidate samples (allowlist disabled). Phase 4 acceptance numbers may not appear anywhere in the review packet.
6. **Scope sweep**: grep the latest collection commands and shell history for any flag that would enable Phase 0 or Phase 4 control. None permitted.
7. **Scaffold off-by-default check**: confirm `vla_pick_place.py --help` still shows `--phase1-adapter-control` defaulting to off and that a launch without that flag does not log `adapter_control_applied=1`.

Pass/fail interpretation: every item must PASS. Any single FAIL keeps the project in observer/dry-run-only mode and triggers a fresh recollection or a redesign discussion, not a threshold loosening.

### AC — Phase 1 control envelope (design only, not yet implemented)

Numeric envelope, all stricter than the Round 10 dry-run maxima:

| Constraint | Value | Source |
| --- | --- | --- |
| Max single-step Cartesian delta norm (Phase 1 only) | `0.030 m` | Half of the Round 10 dry-run cap of `0.05 m`; below the dry-run mean of `0.044 m` so most steps fall back to scripted intentionally. |
| Max XY workspace radius from cube spawn region | `0.30 m` | Existing cube/target XY ranges plus margin; reject any goal outside this disk. |
| Z range (Phase 1 transport altitude) | `[0.05 m, 0.40 m]` above table plane | Phase 1 is approach above the cube; below `0.05 m` is grasp territory and out of scope. |
| Per-launch live-control step budget (`live-once` mode) | `1` step | Single update; subsequent Phase 1 steps revert to scripted. |
| Per-run live-control invocation count | `≤ 1` | One `live-once` invocation per scripted run. |
| Stale VLA timeout | `1.5 s` from query start | If response not back, fall back. |
| VLA query age allowed at apply time | `≤ 0.5 s` | Older responses fall back. |
| Adapter readiness | required `True` | `adapter_ready != 1` falls back. |
| Adapter `delta_norm` finite check | `np.isfinite` on every component | NaN/Inf falls back. |
| Phase guard | `current_phase == 1` at apply time | Phase mismatch aborts. |
| Allowlist guards | `--openvla-enabled-phases 1` AND `--adapter-enabled-phases 1` AND `--phase1-adapter-control 1` | Already enforced at startup in code. |
| Gripper command source | scripted only | Live mode never writes gripper joint targets. |
| Operator arm flag | required true at launch | A future `--phase1-control-require-operator-arm 1` flag must be present and acknowledged. |
| Default fallback | scripted forward step | Existing `forward(args.ik_method)` behavior. |

Implementation notes:

- The scaffold now enforces all of the AC envelope items via `check_phase1_control_envelope` in `vla_pick_place.py`: stricter live `max-delta` (default `0.030 m`), XY workspace bounds, Z range, per-launch apply budget, VLA latency-based query-age check, finite-value check on goal/delta, and an operator-arm gate (`--phase1-control-operator-arm 1` required by default). All preconditions also remain in the startup block (allowlists, dry-run-off, adapter-config-loaded).
- Workspace and altitude checks are applied to the proposed adapter goal itself, not just the delta, to reject teleport-style proposals.
- A "shadow" mode (envelope-validated dry-run that intentionally never applies) is not a separate code path; today it is approximated by `--adapter-dry-run 1 --phase1-adapter-control 0`. A dedicated shadow flag that runs the envelope check and logs the would-be-applied/would-be-rejected outcome is left for a future iteration.

### AD — Abort, fallback, and rollback rules

Abort (run ends, simulator stops, no further Phase 1 live control until a new review):

- OpenVLA HTTP timeout, transport error, or non-2xx response.
- VLA action not 7D, contains NaN/Inf, or has malformed fields.
- Adapter config missing or incompatible at runtime.
- Phase mismatch at apply time (current phase ≠ 1).
- Workspace, altitude, or `max-delta` envelope violation in `live-once` mode (in `shadow` mode this becomes a fallback, not an abort).
- Camera read returns empty frame or stale timestamp on the Phase 1 sampling step.
- Simulator instability: PhysX warning escalation, joint divergence, gripper unexpected open/close.
- Final-placement gate miss in the same run before Phase 1 lives (i.e., if Phase 0 is already off-track, abort live mode for the rest of the run).
- Operator abort (keyboard interrupt, kill signal).

Fallback (scripted controller continues this run, live authority disabled for the rest of the run):

- Adapter `accepted=0` for any reason in dry-run/shadow.
- Adapter readiness false.
- Stale VLA response (older than the query-age threshold).
- Workspace, altitude, or delta envelope violation in `shadow` mode.
- Any analyzer field non-finite.

Rollback (system-level):

- After any abort, the run is preserved but flagged. The scaffold must not re-arm Phase 1 live in the same Isaac Sim session; relaunch is required.
- After any fallback, the same Isaac session may continue, but `live-once` invocations remain consumed for the run.
- Two consecutive abort events across runs trigger a Round 11 redesign rather than another `live-once` retry.
- If any envelope constant is widened, a fresh full dry-run gate must pass first.

Implementation notes:

- Operator abort already works via the simulator close/keyboard-interrupt path. The scaffold relies on `adapter-control-fallback {scripted, abort}`, which already implements the basic fallback/abort split per step; the per-run "no re-arm" rule and consecutive-abort policy require a small launcher state machine and have not been coded.

### AE — Lead review packet template

Include all of the following before requesting a lead review for any Phase 1 `live-once` proposal:

1. **Header**: round (`11`), candidate phase (`1`), proposal mode (`shadow` or `live-once`), date, operator, expected duration, fallback flag.
2. **Repository state**: `git rev-parse HEAD`, `git status --porcelain`, `git diff --stat origin/main...HEAD`, list of changed files since the last accepted dry-run gate.
3. **Scaffold proof**: pasted output of `vla_pick_place.py --help` showing `--phase1-adapter-control` default off; pasted snippet of startup preconditions in `main()` that refuse mismatched configs.
4. **Pre-live evidence (AB)**: command lines used for fast/medium/full collections; analyzer commands; pass/fail line for each gate; dataset locations (uncommitted).
5. **Numeric summary (Phase 1 only)**: runs completed, balanced label counts, VLA success counts, latency p50/p95/max, empty-camera count, scripted final-placement mean/max, adapter accepted/rejected counts, delta p50/p95/max, error p50/p95/max.
6. **Worst rows**: top `5` worst Phase 1 adapter rows by error and by delta norm, with `run_id`, `phase`, `target_label`, `adapter_delta`, `adapter_error_to_scripted`, `adapter_rejected_reason`.
7. **Phase 4 negative**: confirmation that the same dataset shows Phase 4 as `0` candidate samples or the allowlist-disabled rejection reason.
8. **Envelope reconciliation**: table of AC envelope values vs. the observed Round 10 dry-run maxima; explicit margin for each row.
9. **Risk assessment**: known failure modes with the proposed mitigation; what is *not* mitigated; explicit list of cases where the scaffold falls back vs. aborts.
10. **Go/no-go ask**: one of `continue dry-run`, `tighten and recollect`, `request shadow review`, `request live-once review`. Each option lists the scope of the next experiment.
11. **Confirmation**: explicit one-liner that no live-control code path defaults on, no Phase 0 / Phase 4 control is being requested, and no generated log data is staged for commit.

Acceptance: lead may approve or reject the packet. Approval is for the named experiment only and does not extend to subsequent runs without a fresh packet.

## Runbook: Next Dry-Run Adapter Collection

Purpose:

- Collect a fresh scripted-control Isaac Sim run with OpenVLA and the affine adapter in dry-run mode.
- Keep scripted Franka control as the only motion authority.
- Use logged adapter proposals to decide whether the adapter is safe enough for further offline iteration.

Environment:

```powershell
cd C:\Users\Akshit\Projects\isaacsim\_build\windows-x86_64\release
$env:OPENVLA_LOG_DIR="C:\Users\Akshit\Projects\isaacsim\vla_calib_logs_phase1_adapter_dryrun_round10"
$env:OPENVLA_SERVER_URL="http://localhost:8000/act"
$env:OPENVLA_ENABLED="1"
$env:OPENVLA_QUERY_EVERY="10"
$env:OPENVLA_TIMEOUT="60"
$env:OPENVLA_ADAPTER_CONFIG="C:\Users\Akshit\Projects\isaacsim\vla_calib_logs_balanced_real\affine_adapter_config.json"
$env:OPENVLA_ADAPTER_DRY_RUN="1"
$env:OPENVLA_ADAPTER_MAX_DELTA="0.05"
$env:OPENVLA_ADAPTER_ENABLED_PHASES="1"
```

Command:

```powershell
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --device cuda --ik-method damped-least-squares --runs 15 --seed 2039 --target-labels "red target,green target,blue target" --target-label-mode balanced --runs-per-label 5 --instruction-template "pick up the blue cube and place it on the {target_label}" --adapter-dry-run 1 --adapter-max-delta 0.05 --adapter-enabled-phases 1
```

Expected outputs:

- `vla_calib_logs_phase1_adapter_dryrun_round10\calibration.csv`
- Scripted pick-place phases continue normally in console output.
- CSV includes VLA success/action/latency fields.
- CSV includes Phase 1 adapter dry-run fields: readiness, acceptance, rejection reason, proposed goal, proposed delta, delta norm, and distance to scripted goal.
- Phase 4 may still have VLA observations, but adapter candidate fields should be disabled by the Phase 1 allowlist.

Pass/fail interpretation:

- Pass: all scripted runs complete, target labels are balanced, VLA query coverage is adequate, adapter fields are populated, accepted deltas stay within `0.05 m`, and adapter-to-scripted error has no large repeated outliers.
- Fail: missing/empty CSV, poor VLA success coverage, empty camera frames, high latency, repeated adapter rejections, large proposed deltas, large adapter-to-scripted errors, or any sign that adapter dry-run changed gripper/control behavior.
- Safety rule: failure means do not enable live control; keep collecting/debugging in observer-only mode.

## Runbook: Phase 1-Only Scaffold Validation Commands

Purpose:

- Validate a future Phase 1-only control scaffold without weakening the current safety boundary.
- Keep the current dry-run commands runnable today; treat live-control commands below as the exact proposed CLI contract for a later implementation review.
- Phase 1 is the only candidate phase. Phase 0 and Phase 4 must stay scripted/blocked in commands, reports, gates, and review notes.

Common environment:

```powershell
cd C:\Users\Akshit\Projects\isaacsim\_build\windows-x86_64\release
$env:OPENVLA_SERVER_URL="http://localhost:8000/act"
$env:OPENVLA_ENABLED="1"
$env:OPENVLA_TIMEOUT="60"
$env:OPENVLA_ADAPTER_CONFIG="C:\Users\Akshit\Projects\isaacsim\vla_calib_logs_balanced_real\affine_adapter_config.json"
$env:OPENVLA_ADAPTER_MAX_DELTA="0.05"
$env:OPENVLA_ADAPTER_ENABLED_PHASES="1"
$env:OPENVLA_ENABLED_PHASES="1"
$env:OPENVLA_SAVE_IMAGES="0"
$env:OPENVLA_QUERY_EVERY="10"
```

Fast smoke dry-run:

```powershell
$env:OPENVLA_LOG_DIR="C:\Users\Akshit\Projects\isaacsim\vla_calib_logs_phase1_scaffold_smoke_dryrun"
$env:OPENVLA_QUERY_EVERY="20"
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --headless 1 --camera-resolution 160 160 --device cuda --ik-method damped-least-squares --runs 3 --seed 2051 --target-labels "red target,green target,blue target" --target-label-mode balanced --runs-per-label 1 --instruction-template "pick up the blue cube and place it on the {target_label}" --openvla-enabled-phases 1 --openvla-save-images 0 --adapter-dry-run 1 --adapter-max-delta 0.05 --adapter-enabled-phases 1
```

Fast smoke gates:

```powershell
cd C:\Users\Akshit\Projects\isaacsim
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\analyze_vla_calibration.py --csv vla_calib_logs_phase1_scaffold_smoke_dryrun\calibration.csv --required-phases 1 --min-runs 3 --min-label-count 3 --min-runs-per-label 1 --min-query-successes-per-phase 12 --max-empty-camera-frames 0 --max-vla-latency-p95-ms 4000
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\analyze_vla_adapter_dryrun.py --csv vla_calib_logs_phase1_scaffold_smoke_dryrun\calibration.csv --phase 1 --required-phases 1 --min-adapter-samples-per-phase 12 --require-ready --min-accepted-rate 0.90 --min-vla-success-rate 1.0 --max-adapter-delta-p95 0.052 --max-adapter-delta-max 0.055 --max-adapter-error-p95 0.065 --max-adapter-error-max 0.07 --worst-rows 3
```

Medium dry-run:

```powershell
cd C:\Users\Akshit\Projects\isaacsim\_build\windows-x86_64\release
$env:OPENVLA_LOG_DIR="C:\Users\Akshit\Projects\isaacsim\vla_calib_logs_phase1_scaffold_medium_dryrun"
$env:OPENVLA_QUERY_EVERY="10"
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --headless 1 --camera-resolution 160 160 --device cuda --ik-method damped-least-squares --runs 9 --seed 2052 --target-labels "red target,green target,blue target" --target-label-mode balanced --runs-per-label 3 --instruction-template "pick up the blue cube and place it on the {target_label}" --openvla-enabled-phases 1 --openvla-save-images 0 --adapter-dry-run 1 --adapter-max-delta 0.05 --adapter-enabled-phases 1
```

Medium gates:

```powershell
cd C:\Users\Akshit\Projects\isaacsim
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\analyze_vla_calibration.py --csv vla_calib_logs_phase1_scaffold_medium_dryrun\calibration.csv --required-phases 1 --min-runs 9 --min-label-count 3 --min-runs-per-label 3 --min-query-successes-per-phase 36 --max-empty-camera-frames 1 --max-vla-latency-p95-ms 4000 --final-distance-threshold 0.08 --min-success-like-runs 9 --max-final-distance 0.08
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\analyze_vla_adapter_dryrun.py --csv vla_calib_logs_phase1_scaffold_medium_dryrun\calibration.csv --phase 1 --required-phases 1 --min-adapter-samples-per-phase 36 --require-ready --min-accepted-rate 0.90 --min-vla-success-rate 0.98 --max-adapter-delta-p95 0.052 --max-adapter-delta-max 0.055 --max-adapter-error-p95 0.065 --max-adapter-error-max 0.07 --worst-rows 5
```

Full dry-run gate:

```powershell
cd C:\Users\Akshit\Projects\isaacsim\_build\windows-x86_64\release
$env:OPENVLA_LOG_DIR="C:\Users\Akshit\Projects\isaacsim\vla_calib_logs_phase1_scaffold_full_dryrun"
$env:OPENVLA_QUERY_EVERY="10"
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --headless 1 --camera-resolution 160 160 --device cuda --ik-method damped-least-squares --runs 15 --seed 2053 --target-labels "red target,green target,blue target" --target-label-mode balanced --runs-per-label 5 --instruction-template "pick up the blue cube and place it on the {target_label}" --openvla-enabled-phases 1 --openvla-save-images 0 --adapter-dry-run 1 --adapter-max-delta 0.05 --adapter-enabled-phases 1
```

Full gates:

```powershell
cd C:\Users\Akshit\Projects\isaacsim
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\analyze_vla_calibration.py --csv vla_calib_logs_phase1_scaffold_full_dryrun\calibration.csv --required-phases 1 --min-runs 15 --min-label-count 3 --min-runs-per-label 5 --min-query-successes-per-phase 60 --max-empty-camera-frames 1 --max-vla-latency-p95-ms 4000 --final-distance-threshold 0.08 --min-success-like-runs 15 --max-final-distance 0.08
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\analyze_vla_adapter_dryrun.py --csv vla_calib_logs_phase1_scaffold_full_dryrun\calibration.csv --phase 1 --required-phases 1 --min-adapter-samples-per-phase 60 --require-ready --min-accepted-rate 0.90 --min-vla-success-rate 0.98 --max-adapter-delta-p95 0.052 --max-adapter-delta-max 0.055 --max-adapter-error-p95 0.065 --max-adapter-error-max 0.07 --worst-rows 5
```

Live-control scaffold commands (envelope flags now implemented; live-once still requires lead approval before any run):

The Phase 1 scaffold flags below correspond to the `vla_pick_place.py` CLI. Live-once is gated by the AB checklist passing and a fresh lead review; do not launch a live-once command before then. Shadow mode (envelope-validated dry-run with no robot authority) is not yet a separate code path — it can be approximated today by combining `--adapter-dry-run 1` with `--phase1-adapter-control 0` so the proposal is logged but never applied.

```powershell
# First live-once scaffold smoke, only after full dry-run gates AND lead review pass.
# Operator must explicitly arm via --phase1-control-operator-arm 1 before this will start.
cd C:\Users\Akshit\Projects\isaacsim\_build\windows-x86_64\release
$env:OPENVLA_LOG_DIR="C:\Users\Akshit\Projects\isaacsim\vla_calib_logs_phase1_scaffold_smoke_live_once"
$env:OPENVLA_QUERY_EVERY="20"
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --headless 0 --camera-resolution 160 160 --device cuda --ik-method damped-least-squares --runs 3 --seed 2062 --target-labels "red target,green target,blue target" --target-label-mode balanced --runs-per-label 1 --instruction-template "pick up the blue cube and place it on the {target_label}" --openvla-enabled-phases 1 --openvla-save-images 0 --adapter-dry-run 0 --adapter-max-delta 0.05 --adapter-enabled-phases 1 --phase1-adapter-control 1 --adapter-control-fallback scripted --phase1-control-max-delta 0.03 --phase1-control-workspace-x 0.20 0.80 --phase1-control-workspace-y -0.50 0.50 --phase1-control-z-range 0.05 0.40 --phase1-control-max-applies 1 --phase1-control-vla-max-age-ms 500 --phase1-control-require-operator-arm 1 --phase1-control-operator-arm 1
```

Live-control scaffold review gates:

- Do not run any live-once command until the full dry-run gate above passes on a fresh dataset and lead review explicitly approves the single experiment.
- The scaffold rejects startup unless `--phase1-adapter-control 1`, `--adapter-dry-run 0`, `--openvla-dry-run 0`, OpenVLA is enabled, and Phase 1 is in both `--openvla-enabled-phases` and `--adapter-enabled-phases`. With `--phase1-control-require-operator-arm 1` (default), `--phase1-control-operator-arm 1` is also required.
- Fast smoke is for plumbing only; medium is for regression confidence; full is the only dry-run gate acceptable before a live-control review packet.
- For the first live smoke, use `--headless 0` so the operator can see and abort the run. Any timeout, malformed action, adapter rejection, phase mismatch, unexpected gripper command, final-distance miss, simulator instability, or analyzer failure blocks further live attempts.
- After live smoke, run the same dataset and adapter analyzers where applicable, plus a manual review of scaffold-specific columns: `adapter_control_enabled`, `adapter_control_applied`, `adapter_control_fallback`, and rejection reasons in `adapter_rejected_reason` (which now also reports envelope violations such as `goal Z 0.42 outside [0.05, 0.40]` or `delta_norm 0.040 > phase 1 live cap 0.030`).

## Verification Targets

- Python syntax checks pass for changed standalone scripts/tools.
- Observer script can still run with scripted control unchanged.
- Dataset report can parse existing `vla_calib_logs/calibration.csv`.
- Logs clearly distinguish successful VLA rows, skipped rows, and error rows.

## Completed Swarm Round 1

Merged branches:

- `codex/openvla-baseline-checkpoint`
- `codex/openvla-dataset-report`
- `codex/openvla-randomized-scenes`
- `codex/openvla-query-diagnostics`

New or updated capabilities:

- `vla_pick_place.py` can randomize cube and target positions per scripted observer run.
- `vla_pick_place.py` has VLA diagnostics and controls:
  - `--openvla-enabled` / `OPENVLA_ENABLED`
  - `--openvla-timeout` / `OPENVLA_TIMEOUT`
  - `--openvla-save-images` / `OPENVLA_SAVE_IMAGES`
  - `--openvla-dry-run` / `OPENVLA_DRY_RUN`
  - `vla_latency_ms` CSV column
  - 7D VLA action validation
- `analyze_vla_calibration.py` summarizes calibration CSVs without Isaac imports.
- Scripted Franka control remains unchanged; VLA is still observer/logging only.

Useful commands:

```powershell
cd C:\Users\Akshit\Projects\isaacsim\_build\windows-x86_64\release
$env:OPENVLA_LOG_DIR="C:\Users\Akshit\Projects\isaacsim\vla_calib_logs_next"
.\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --device cuda --ik-method damped-least-squares --runs 3 --seed 123 --openvla-dry-run 1
```

```powershell
cd C:\Users\Akshit\Projects\isaacsim
python source\standalone_examples\api\isaacsim.robot.manipulators\franka\analyze_vla_calibration.py --csv vla_calib_logs_next\calibration.csv
```

Test evidence from implementers:

- Dataset analyzer parsed existing `vla_calib_logs/calibration.csv`: 280 rows, 1 run, 13 VLA successes, 1 VLA error.
- Randomized observer completed 2 D3D12 scripted runs with distinct cube and target XY positions.
- Diagnostics observer completed disabled-VLA, dry-run, and malformed-action tests without breaking scripted simulation.

Current next step:

- Collect a larger randomized observer dataset using fresh log directories.
- Use the analyzer to verify coverage, VLA success rate, image capture, and end-effector goal deltas.
- Only after dataset quality is good, start building an action adapter in dry-run mode.

