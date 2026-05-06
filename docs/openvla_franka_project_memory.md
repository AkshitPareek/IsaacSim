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

- Lead: refine task list, review implementer plans, keep this memory file current.
- Implementer A: add randomized cube and target placement to observer runs.
- Implementer B: add dataset inspection/report tooling for calibration CSV outputs.
- Implementer C: harden OpenVLA query/logging controls and runtime diagnostics without changing robot control.

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

