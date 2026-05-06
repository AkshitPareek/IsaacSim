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

