# CLAUDE_UNITREE_G1.md

Developer reference for Unitree G1 integration in LeRobot. For the user-facing tutorial, see `docs/source/unitree_g1.mdx`.

## Installation

G1 is **not** included in `pip install -e ".[all]"` due to SDK installation complexity. Install separately:

```bash
# 1. Install unitree_sdk2_python (requires CycloneDDS v0.10.2)
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python && pip install -e .

# 2. Install LeRobot G1 extras
pip install -e ".[unitree_g1]"
```

Dependencies: `unitree-sdk2==1.0.1`, `pyzmq`, `onnxruntime`, `pin` (Pinocchio), `meshcat`, `casadi`, `matplotlib`, `pygame`

## Code Structure

```
src/lerobot/
├── robots/unitree_g1/
│   ├── config_unitree_g1.py      # UnitreeG1Config (draccus ChoiceRegistry: "unitree_g1")
│   ├── unitree_g1.py             # UnitreeG1 robot class (inherits Robot ABC)
│   ├── g1_utils.py               # Joint definitions (G1_29_JointIndex, G1_29_JointArmIndex), remote keys
│   ├── g1_kinematics.py          # Pinocchio + CasADi IK solver (G1_29_ArmIK)
│   ├── unitree_sdk2_socket.py    # ZMQ bridge layer (replaces direct DDS for real robot)
│   ├── run_g1_server.py          # On-robot server (DDS <-> ZMQ bridge + optional camera server)
│   ├── gr00t_locomotion.py       # GR00T locomotion controller (ONNX, 50Hz)
│   └── holosoma_locomotion.py    # Holosoma locomotion controller (ONNX, 200Hz)
│
├── teleoperators/unitree_g1/
│   ├── config_unitree_g1.py      # UnitreeG1TeleoperatorConfig
│   ├── unitree_g1.py             # UnitreeG1Teleoperator (exoskeleton + wireless remote)
│   ├── exo_calib.py              # Exoskeleton calibration (Matplotlib GUI)
│   ├── exo_ik.py                 # Exoskeleton FK -> G1 arm IK
│   └── exo_serial.py             # ExoskeletonArm serial communication (115200 baud, 16ch ADC)
│
├── cameras/zmq/                  # ZMQ remote cameras (used by G1)
│   ├── camera_zmq.py             # ZMQCamera (ZMQ SUB, receives base64 JPEG)
│   ├── configuration_zmq.py      # ZMQCameraConfig
│   └── image_server.py           # ImageServer (on-robot ZMQ PUB)
│
tests/
├── robots/test_unitree_g1.py                      # Robot unit tests
└── teleoperators/test_unitree_g1_teleoperator.py   # Teleoperator unit tests
```

## Joint Definition (29 DOF)

| Index | Joints | Group |
|-------|--------|-------|
| 0-5 | LeftHipPitch/Roll/Yaw, LeftKnee, LeftAnklePitch/Roll | Left leg |
| 6-11 | RightHipPitch/Roll/Yaw, RightKnee, RightAnklePitch/Roll | Right leg |
| 12-14 | WaistYaw/Roll/Pitch | Waist |
| 15-21 | LeftShoulder P/R/Y, LeftElbow, LeftWrist R/P/Y | Left arm |
| 22-28 | RightShoulder P/R/Y, RightElbow, RightWrist R/P/Y | Right arm |

Arm subset `G1_29_JointArmIndex`: indices 15-28 (14 joints). When a locomotion controller is active, only these are controlled by the teleoperator.

## Communication Architecture

### Simulation mode (`is_simulation=true`)
Uses Unitree SDK DDS directly via `ChannelFactoryInitialize(0, "lo")`. MuJoCo environment is loaded from HF Hub `lerobot/unitree-g1-mujoco`.

### Real robot mode (`is_simulation=false`)
```
Client (your machine)                Robot (192.168.123.164)
UnitreeG1 <-> ZMQ Socket           run_g1_server.py <-> DDS (Unitree SDK)
  PUSH -> port 6000 (commands) ->     -> rt/lowcmd
  SUB  <- port 6001 (state)   <-     <- rt/lowstate
  SUB  <- port 5555 (camera)  <-     <- ImageServer
```

- State rate: ~500Hz DDS on robot -> 250Hz sampling on client
- Command format: JSON over ZMQ
- Safe disconnect: sends zero-gain commands (`kp=kd=tau=0`) to make joints passive

## Locomotion Controllers

Specified via `--robot.controller=GrootLocomotionController` or `HolosomaLocomotionController`. The controller owns legs + waist (indices 0-14); arms (indices 15-28) remain under teleoperator control.

| Controller | Rate | Model Source | Notes |
|------------|------|-------------|-------|
| `GrootLocomotionController` | 50Hz | `nepyope/GR00T-WholeBodyControl_g1` | Dual-model switching (Balance/Walk), supports waist height adjustment |
| `HolosomaLocomotionController` | 200Hz | `nepyope/holosoma_locomotion` | Single model (fastsac/ppo), hides arm state to prevent destabilization |

Joystick mapping: `lx->lateral (negated)`, `ly->forward`, `rx->rotation (negated)`, R1/R2->raise/lower waist

## Teleoperator Modes

### Remote-only mode (no serial ports configured)
- Action output: 4 joystick axes (`remote.lx/ly/rx/ry`)
- Use case: locomotion-only control

### Exoskeleton mode (left/right_arm_config.port set)
- Action output: 14 arm joint angles + 4 joystick axes
- Pipeline: exoskeleton hall-effect sensors -> FK -> IK (Pinocchio + CasADi IPOPT)
- IK output is smoothed with weighted moving average filter (window 4, weights [0.4, 0.3, 0.2, 0.1])
- Wireless remote takes priority over the mini-joystick on the exoskeleton

## Observation and Action Spaces

### Observations (`get_observation`)
- `{joint_name}.q` x 29 (joint positions) — this is what `observation_features` exposes
- Also collected internally but not exposed as features: joint velocity `.dq`, torque `.tau`, IMU (gyro/accel/quat/rpy)
- Camera frames (one per configured ZMQ camera)

### Actions (`action_features`)
- Without controller: `{joint_name}.q` x 29
- With controller: `{arm_joint}.q` x 14 + `remote.lx/ly/rx/ry` x 4

## Data Collection Workflow

```bash
# 1. Start the server on the robot
ssh unitree@192.168.123.164
python src/lerobot/robots/unitree_g1/run_g1_server.py --camera

# 2. Record from client (exoskeleton + locomotion controller)
lerobot-record \
  --robot.type=unitree_g1 \
  --robot.is_simulation=false \
  --robot.robot_ip=192.168.123.164 \
  --robot.controller=HolosomaLocomotionController \
  --robot.cameras='{"head": {"type": "zmq", "server_address": "192.168.123.164", "port": 5555, "camera_name": "head_camera", "width": 640, "height": 480, "fps": 30}}' \
  --teleop.type=unitree_g1 \
  --teleop.left_arm_config.port=/dev/ttyACM1 \
  --teleop.right_arm_config.port=/dev/ttyACM0 \
  --teleop.id=exo \
  --dataset.repo_id=<your-username>/<dataset-name> \
  --dataset.single_task="task description" \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --dataset.push_to_hub=true \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2

# 3. Remote-only mode (omit exoskeleton ports)
lerobot-record \
  --robot.type=unitree_g1 \
  --robot.is_simulation=false \
  --robot.robot_ip=192.168.123.164 \
  --robot.controller=HolosomaLocomotionController \
  --teleop.type=unitree_g1 \
  --teleop.id=wbc_unitree \
  --dataset.repo_id=<your-username>/<dataset-name> \
  --dataset.single_task="walking task"
```

Data is saved in LeRobotDataset format (MP4 videos + Parquet state/action files) and can be uploaded to HuggingFace Hub.

## Training and Inference

```bash
# Train (pi0-fast policy recommended)
lerobot-train \
  --dataset.repo_id=<your-username>/<dataset-name> \
  --policy.type=pi05 \
  --policy.pretrained_path=lerobot/pi05_base \
  --batch_size=32 --steps=3000

# Inference (RTC mode)
python examples/rtc/eval_with_real_robot.py \
  --policy.path=<your-username>/<model-repo> \
  --robot.type=unitree_g1 \
  --robot.controller=HolosomaLocomotionController \
  --rtc.enabled=true
```

## Testing

```bash
# Tests are skipped if unitree_sdk2 is not installed
pytest tests/robots/test_unitree_g1.py -vv
pytest tests/teleoperators/test_unitree_g1_teleoperator.py -vv
```

Tests use mocks in place of the real SDK. Coverage includes: joint definition correctness, config initialization, observation/action features, connect/disconnect logic, remote controller parsing.

## Known Limitations

1. **G1 robot calibration is not implemented** — `calibrate()` is a no-op, `is_calibrated` always returns `True`
2. **Exoskeleton wrist pitch/yaw not calibrated** — marked as TODO in `exo_calib.py`
3. **Single robot per process** — `unitree_sdk2_socket.py` uses module-level ZMQ globals
4. **Only 29-DOF supported** — docs mention 23-DOF but it is not implemented in code
5. **ZMQ camera `find_cameras()` not implemented** — raises `NotImplementedError`
6. **Gravity compensation is experimental** — `gravity_compensation` defaults to `False`

## Reference Dataset

- `nepyope/unitree_box_move_blue_full` — example dataset on HF Hub
