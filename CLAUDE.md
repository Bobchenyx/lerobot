# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LeRobot is a PyTorch-based robotics library by Hugging Face for imitation learning and reinforcement learning. It provides policies (neural network models), datasets, simulation environments, and real-world robot control. Source code lives under `src/lerobot/`.

## Common Commands

### Installation
```bash
pip install -e ".[dev,test]"          # Development install
pip install -e ".[dev,test,aloha,pusht,xarm]"  # With simulation envs
```

### Testing
```bash
python -m pytest tests -v                    # Run all tests
python -m pytest tests/path/test_file.py -v  # Run a single test file
python -m pytest tests/path/test_file.py::test_name -v  # Run a single test
```
Tests require `git lfs pull` for test artifacts in `tests/artifacts/`.

### Linting & Formatting
```bash
pre-commit run --all-files    # Run all pre-commit hooks (ruff, typos, bandit, etc.)
ruff check src/               # Lint only
ruff format src/               # Format only
```
Pre-commit hooks include: ruff (format + lint), typos, pyupgrade, bandit (security), gitleaks, zizmor (GitHub Actions).

### Training & Evaluation
```bash
python -m lerobot.scripts.train --policy.type=act --dataset.repo_id=lerobot/aloha_sim_transfer_cube_human --env.type=aloha
python -m lerobot.scripts.eval --policy.path=outputs/train/.../checkpoints/last/pretrained_model
```

### CLI Entry Points
`lerobot-train`, `lerobot-eval`, `lerobot-record`, `lerobot-replay`, `lerobot-teleoperate`, `lerobot-calibrate`, `lerobot-find-cameras`, `lerobot-find-port`, `lerobot-setup-motors`

## Architecture

### Configuration System
Uses **draccus** (not hydra) for config parsing. All configs are Python dataclasses with CLI override support (`--policy.type=act --batch_size=32`). Key config classes:

- `TrainPipelineConfig` (`configs/train.py`) — top-level training config, composes dataset, policy, env, eval, optimizer, wandb configs
- `PreTrainedConfig` (`configs/policies.py`) — base policy config using `draccus.ChoiceRegistry` for polymorphic dispatch via `--policy.type=<name>`
- `RobotConfig` (`robots/config.py`) — base robot config, also uses `ChoiceRegistry`
- `EnvConfig` (`envs/configs.py`) — simulation environment config

The `configs/parser.py` module provides a custom `wrap` decorator (used on `main()` functions) that handles CLI parsing, config loading from pretrained paths (`--policy.path=`), and plugin discovery.

### Policy System
All policies extend `PreTrainedPolicy` (`policies/pretrained.py`), which extends `nn.Module` with HuggingFace Hub integration (save/load via safetensors).

Each policy lives in its own subpackage under `policies/` with two key files:
- `configuration_<name>.py` — dataclass config (extends `PreTrainedConfig`)
- `modeling_<name>.py` — model implementation (extends `PreTrainedPolicy`)

Available policies: **ACT**, **Diffusion**, **TDMPC**, **VQ-BeT**, **PI0**, **PI0FAST**, **SAC**, **SmolVLA**

`policies/factory.py` dispatches by name string to the correct class. Policies include built-in normalization (`policies/normalize.py`) and optimizer/scheduler presets.

### Dataset System
`LeRobotDataset` (`datasets/lerobot_dataset.py`) wraps HuggingFace datasets with robot-specific features:
- Temporal indexing via `delta_timestamps` (retrieve multiple frames relative to an index)
- Video frames stored as mp4, decoded on-the-fly
- Episode-aware sampling (`datasets/sampler.py`)
- Statistics for normalization
- Seamless Hub upload/download

### Robot & Hardware Layer
`robots/robot.py` defines the abstract `Robot` base class. Concrete implementations (Koch, SO100, SO101, LeKiwi, etc.) live in subpackages under `robots/`. Hardware abstractions:
- `motors/` — Dynamixel and Feetech servo SDKs
- `cameras/` — OpenCV and Intel RealSense
- `teleoperators/` — leader arm / teleoperation devices

### Environment Integration
Simulation environments are external gym packages (`gym-aloha`, `gym-pusht`, `gym-xarm`) installed as optional dependencies. `envs/factory.py` creates vectorized gym environments for parallel evaluation.

### Key Registries
`lerobot/__init__.py` maintains lists: `available_policies`, `available_robots`, `available_cameras`, `available_motors`, `available_envs`, `available_datasets_per_env`. Update these when adding new components.

## Code Style

- Ruff with line length 110, target Python 3.10
- Import sorting: `isort` via ruff with `lerobot` as first-party
- No type checker enforced yet (mypy config exists but is commented out)
- Tests use pytest with fixtures defined in `tests/fixtures/`
