# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LeRobot is a PyTorch library for real-world robotics by Hugging Face. It provides unified abstractions for robot hardware, datasets, policies (imitation learning and RL), simulation environments, and teleoperation. Python 3.12+ required.

## Build & Install

```bash
pip install -e ".[dev,test]"       # Development install
pip install -e ".[all]"            # All extras (policies, envs, hardware)
# Or with uv (preferred in CI):
uv sync --extra all
```

Git LFS is needed for test artifacts: `git lfs install && git lfs pull`

## Common Commands

### Testing
```bash
pytest tests -vv --maxfail=10                          # Full test suite
pytest tests/path/to/test_file.py -vv                  # Single file
pytest tests/path/to/test_file.py::test_function -vv   # Single test
make test-end-to-end                                   # E2E policy train+eval
make DEVICE=cuda test-end-to-end                       # E2E on GPU
```

Key env vars: `LEROBOT_TEST_DEVICE` (cpu/cuda), `MUJOCO_GL=egl` (headless rendering), `HF_USER_TOKEN` (gated models).

Tests use skip decorators from `tests/utils.py`: `@require_cuda`, `@require_cpu`, `@require_env`, `@require_package`, `@require_hf_token`.

### Linting & Formatting
```bash
pre-commit run --all-files    # Run all checks (ruff, mypy, typos, bandit, etc.)
ruff format .                 # Format only
ruff check --fix .            # Lint with auto-fix
```

Ruff config: line length 110, double quotes, Google-style docstrings. Rule sets: E, W, F, I, B, C4, T20, N, UP, SIM.

mypy is enforced only for: `lerobot.envs`, `lerobot.configs`, `lerobot.optim`, `lerobot.model`, `lerobot.cameras`, `lerobot.motors`, `lerobot.transport`. All other modules have `ignore_errors = true`.

### Training & Evaluation
```bash
lerobot-train --policy.type=act --dataset.repo_id=lerobot/aloha_mobile_cabinet
lerobot-eval --policy.path=<hub_or_local_path> --env.type=pusht
```

## Hardware-Specific Guides

- **Unitree G1**: See [CLAUDE_UNITREE_G1.md](CLAUDE_UNITREE_G1.md) for G1 developer reference (code structure, communication architecture, joint definitions, data collection workflow, known limitations)

## Architecture

### Source Layout

All source code is under `src/lerobot/` (setuptools src layout). Key directories:

- `policies/` — Policy implementations (ACT, Diffusion, Pi0, SmolVLA, SAC, VQ-BeT, TDMPC, etc.)
- `datasets/` — LeRobotDataset (synchronized MP4 videos + Parquet state/action files)
- `envs/` — Simulation environment configs and factories (Aloha, PushT, Libero, MetaWorld)
- `robots/` — Robot hardware abstraction (SO100, Koch, LeKiwi, Reachy2, etc.)
- `cameras/` — Camera abstraction (OpenCV, RealSense, ZMQ)
- `motors/` — Motor bus abstraction (Dynamixel, Feetech, Damiao, Robstride)
- `teleoperators/` — Teleoperation devices (leader arms, gamepad, keyboard)
- `processor/` — Data processing pipeline framework (pre/post-processing for policies and robots)
- `configs/` — Configuration system (training, eval, policy configs, CLI parser)
- `optim/` — Optimizer and LR scheduler configs/factories
- `scripts/` — All CLI entry points (`lerobot-train`, `lerobot-eval`, `lerobot-record`, etc.)
- `rl/` — Reinforcement learning (HIL-SERL actor/learner, SAC replay buffer)

### Configuration System (draccus, not Hydra)

LeRobot uses **draccus** for configuration via Python dataclasses with `ChoiceRegistry`:

- CLI parsing: `@parser.wrap()` decorator, args like `--policy.type=act --policy.chunk_size=100`
- Type selection: `--policy.type=act` resolves to `ACTConfig` via `draccus.ChoiceRegistry`
- Pretrained loading: `--policy.path=repo/id` loads config from HuggingFace Hub
- Configs serialize to JSON (`config.json`, `train_config.json`)

Config hierarchy for training:
```
TrainPipelineConfig
  ├── DatasetConfig
  ├── EnvConfig          (ChoiceRegistry: "aloha", "pusht", "libero", ...)
  ├── PreTrainedConfig   (ChoiceRegistry: "act", "diffusion", "pi0", ...)
  ├── OptimizerConfig    (ChoiceRegistry: "adam", "adamw", ...)
  ├── LRSchedulerConfig
  ├── EvalConfig
  └── WandBConfig
```

### Key Base Classes and Patterns

**Policies:** All inherit from `PreTrainedPolicy` (`policies/pretrained.py`) which extends `nn.Module + HubMixin`. Must implement `forward()` (training), `select_action()` (inference), `reset()`, `get_optim_params()`. Each policy has a companion config class registered via `@PreTrainedConfig.register_subclass("name")` in its `configuration_*.py` file.

**Hardware ABCs:** `Robot` (`robots/robot.py`), `Camera` (`cameras/camera.py`), `MotorsBusBase` (`motors/motors_bus.py`), `Teleoperator` (`teleoperators/teleoperator.py`) — all follow connect/disconnect/is_connected patterns with their own `ChoiceRegistry`-based configs.

**Processor Pipeline:** `ProcessorStep` (ABC) instances chain into `DataProcessorPipeline` for data transforms. Steps register via `@ProcessorStepRegistry.register("name")`. Canonical data types in `processor/core.py`: `PolicyAction`, `RobotAction`, `EnvAction`, `RobotObservation`, `EnvTransition`.

**Factory functions:** `policies/factory.py`, `datasets/factory.py`, `envs/factory.py` create instances from configs. Policy factory dynamically imports from `modeling_*.py` based on the config's module path.

### Plugin System

External packages can register as LeRobot plugins:
- **Auto-discovery:** Installed packages named `lerobot_robot_*`, `lerobot_camera_*`, `lerobot_teleoperator_*`, `lerobot_policy_*` are auto-imported at startup via `register_third_party_plugins()`.
- **CLI loading:** `--env.discover_packages_path=my.package` imports and registers external packages.

### Data Flow

```
Training:  dataset → dataloader → preprocessor → policy.forward() → loss
Inference: env/robot obs → preprocessor → policy.select_action() → postprocessor → env.step()/robot.send_action()
```

### Adding a New Policy

Each policy lives in `policies/<name>/` with three files following a naming convention:
- `configuration_<name>.py` — Config dataclass with `@PreTrainedConfig.register_subclass("<name>")`
- `modeling_<name>.py` — Policy class inheriting `PreTrainedPolicy`
- `processor_<name>.py` — Pre/post-processing pipeline factory
