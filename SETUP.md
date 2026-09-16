# Mobile Fetch: laptop setup and next steps

## Current status

Based on checks through September 15, 2026:

- [x] Ubuntu 22.04 with ROS 2 Humble installed.
- [x] System Python 3.10.12 imports `rclpy`.
- [x] Project directory: `~/projects/mobile_fetch`.
- [x] Virtual environment: `mobile-fetch` inside the project directory.
- [x] MuJoCo 3.13.0 and NumPy 1.26.4 installed in the environment.
- [x] A ROS node starts successfully from the same environment as MuJoCo.
- [x] `.gitignore` excludes the environment and generated outputs.
- [x] Dependency check passes with the activation helper.
- [x] Resolve NVIDIA NVML driver/library mismatch.
- [ ] Verify the MuJoCo viewer opens.
- [x] Identify GPU model, VRAM, and driver.
- [x] Install MuJoCo Warp 3.13.0 and Warp 1.17.0; detect `cuda:0`.
- [x] Verify batched free-fall physics on the GPU: 16 worlds, 50 steps each.

The laptop reports 15 GiB RAM and an NVIDIA GeForce RTX 4060 Laptop GPU with 8188 MiB VRAM. NVIDIA kernel driver and `nvidia-smi` report version 580.178.04. The previous driver mismatch is resolved; section 3 is retained for troubleshooting.

## 1. Activate the project environment

Run this in each new project terminal:

```bash
cd ~/projects/mobile_fetch
source scripts/activate.bash
```

`PYTHONNOUSERSITE=1` excludes packages from `~/.local`, including the previous MuJoCo 3.1.5 installation. It must be set again in new terminals. The environment uses the system Python and exposes system packages for ROS compatibility.

The activation helper sets this variable automatically. Source the helper rather than executing it, so its settings apply to your current terminal.

### Conflicts from unrelated packages

If `pip check` reports conflicts involving Gym, cvxpy, f1tenth-gym, Open3D, pyrender, or scikit-image, check whether user-installed packages have leaked into the environment:

```bash
source scripts/activate.bash
python -c "import site; print('User site enabled:', site.ENABLE_USER_SITE)"
python -m pip check
```

Expected: `User site enabled: False` and `No broken requirements found.` In the inspected environment, excluding the user site resolved the reported conflicts without package changes. Do not install or downgrade those unrelated dependencies to fix this project.

## 2. Fix the Python dependency check

With the environment active:

```bash
python -m pip install cffi
python -m pip check
```

Expected output: `No broken requirements found.` If other conflicts appear, save the output before changing additional packages. PyNaCl may be inherited from system packages; it is not a core project dependency.

## 3. Resolve the NVIDIA driver mismatch

The reported error was:

```text
Failed to initialize NVML: Driver/library version mismatch
NVML library version: 580.178
```

This indicates a mismatch between the loaded kernel driver and the NVIDIA user-space library. A driver update without a subsequent restart is one possible cause.

### First attempt: restart

1. Record the currently loaded driver version:

   ```bash
   cat /proc/driver/nvidia/version
   ```

2. Save all work and restart the laptop using Ubuntu's power menu.
3. Open a terminal and run:

   ```bash
   nvidia-smi
   ```

This check does not require the Python environment. NVIDIA documents restarting after driver updates as a remedy for this mismatch: [NVIDIA explanation](https://forums.developer.nvidia.com/t/nvml-error-driver-library-version-mismatch/218322/2).

### If the restart fixes it

Record the GPU details:

```bash
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv
```

### If the mismatch persists

Collect these diagnostics:

```bash
cat /proc/driver/nvidia/version
uname -r
modinfo -F version nvidia
dpkg -l | grep -E 'nvidia-driver|nvidia-dkms|nvidia-utils|libnvidia-compute'
```

Use the results to identify mismatched packages before reinstalling drivers. Hold GPU package installation until `nvidia-smi` works.

## 4. Verify ROS and MuJoCo together

Activate the environment using section 1, then run:

```bash
python - <<'PY'
import sys
import rclpy
import mujoco
import numpy

print('Python:', sys.executable)
print('MuJoCo:', mujoco.__version__)
print('MuJoCo location:', mujoco.__file__)
print('NumPy:', numpy.__version__)

rclpy.init()
node = rclpy.create_node('mobile_fetch_setup_check')
node.get_logger().info('ROS 2 and MuJoCo imports work')
node.destroy_node()
rclpy.shutdown()
PY
```

Expected versions are MuJoCo 3.13.0 and NumPy 1.26.4. Python and MuJoCo paths should be inside `~/projects/mobile_fetch/mobile-fetch/`.

## 5. Verify graphics and physics

First launch the empty viewer:

```bash
python -m mujoco.viewer
```

Close the window after confirming it opens. This checks graphics startup, not GPU physics.

For a basic physics check, create a small scene:

```bash
cat > models/setup_test.xml <<'XML'
<mujoco model="setup_test">
  <option timestep="0.002"/>
  <worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="3 3 0.1"
          rgba="0.8 0.8 0.8 1"/>
    <body name="box" pos="0 0 0.6">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"
            rgba="0.9 0.2 0.2 1"/>
    </body>
  </worldbody>
</mujoco>
XML

python -m mujoco.viewer --mjcf=models/setup_test.xml
```

Expected: a red cube falls and settles on the floor. Press Space if paused. See the [official viewer documentation](https://mujoco.readthedocs.io/en/stable/python.html).

## 6. Configure GPU parallel simulation after the driver works

The project will use two execution modes:

| Mode | Purpose |
| --- | --- |
| Standard MuJoCo and ROS 2 | Develop and debug one complete robot; physics runs on CPU |
| GPU batch simulation | Run multiple environments for training and evaluation |

MuJoCo Warp is the proposed NVIDIA GPU backend. Installing `mujoco` alone does not enable GPU physics. Its software is open source, but acceleration also requires NVIDIA driver/CUDA components.

Next steps once GPU details are available:

1. Check GPU and driver compatibility with the selected released Warp version.
2. Choose and pin compatible dependencies, using a separate GPU environment if needed to preserve ROS compatibility.
3. Verify that Warp detects a CUDA device.
4. Run a small batch of simple simulations and confirm execution on the GPU.
5. Compare throughput and memory use against CPU execution before increasing batch size.
6. Check robot model feature compatibility before porting the full scene.

### Install and check device detection

The hardware check is complete. Install the selected MuJoCo Warp release while preserving the base dependency pins:

```bash
cd ~/projects/mobile_fetch
source scripts/activate.bash
python -m pip install -r requirements-gpu.txt
python -m pip check

python - <<'PY'
from importlib.metadata import version
import warp as wp
import mujoco_warp

wp.init()
print('MuJoCo Warp:', version('mujoco-warp'))
print('Warp:', version('warp-lang'))
print('Devices:', wp.get_devices())
assert wp.is_cuda_available(), 'CUDA device not available to Warp'
print('CUDA device:', wp.get_device('cuda:0'))
PY
```

Installation and CUDA detection passed on the laptop with MuJoCo Warp 3.13.0 and Warp 1.17.0. These versions are recorded in `requirements-gpu.txt`.

### Verify batched physics

```bash
python scripts/check_gpu_sim.py
```

The script explicitly uses `cuda:0`, simulates 16 spheres in independent worlds from different starting heights, and checks their positions, velocities, and simulated times against free-fall equations. The first run compiles kernels and can take longer. Expected: a `PASS` line for 16 worlds after 50 steps each. This is a correctness check, not a throughput benchmark; contacts and rendering need subsequent checks. A passing run is required before marking batched physics verified.

Verified on the RTX 4060 Laptop GPU: all checks passed, with maximum height error `1.19e-07 m`. GPU access required running outside the restricted execution sandbox.

Reference: [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp), [selected release](https://pypi.org/project/mujoco-warp/3.13.0/).

## 7. Git hygiene

Keep source code, models, configuration, dependency files, and this guide in Git. Keep virtual environments, ROS build outputs, downloaded weights, and generated experiment data excluded.

Verify exclusions and review changes:

```bash
git check-ignore mobile-fetch/pyvenv.cfg
git status --short
```

The first command should print `mobile-fetch/pyvenv.cfg`.

## Reference: creating the environment on a fresh checkout

The current environment already exists. These commands are for a fresh setup:

```bash
sudo apt update
sudo apt install python3-venv python3-pip \
  python3-colcon-common-extensions git libgl1 libglfw3

mkdir -p ~/projects/mobile_fetch
cd ~/projects/mobile_fetch
mkdir -p ros2_ws/src models scripts configs results

source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages mobile-fetch
source mobile-fetch/bin/activate
export PYTHONNOUSERSITE=1
touch mobile-fetch/COLCON_IGNORE

python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip check
```

Use the tracked `.gitignore` and `requirements.txt` from this project. If the inherited PyNaCl dependency warning appears, apply section 2. Removing the old user-installed MuJoCo is unnecessary and may affect its existing `dm_control` installation.

## Project milestones after setup

1. Build a differential-drive base and a small room in MuJoCo.
2. Bridge ROS `/cmd_vel` commands to the simulator; publish simulation time, joint states, and transforms.
3. Add noisy wheel encoders, IMU, and visual landmarks; implement and evaluate sensor fusion.
4. Implement path planning and a Python path-tracking controller.
5. Integrate an arm, RGB-D perception, and stationary pick-and-place.
6. Coordinate fetch-and-place tasks with retries and completion checks.
7. Collect demonstrations and add a limited language-conditioned picking policy.
8. Evaluate localization error, task success, recovery behavior, and simulation throughput.
