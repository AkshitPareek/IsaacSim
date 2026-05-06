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
- [ ] Generate a larger randomized observer dataset with real OpenVLA responses.
- [x] Add dataset quality gates so bad calibration runs are obvious.
- [x] Add language-conditioned target variants and target labels/colors.
- [x] Design a dry-run action adapter that maps VLA/action signals to bounded Franka waypoints.
- [ ] Validate the adapter offline against logged scripted goals.
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

