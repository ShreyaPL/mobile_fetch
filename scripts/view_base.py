"""View the passive mobile base, or check that it rests stably without a GUI."""

import argparse
from pathlib import Path
import time

import mujoco
import numpy as np


MODEL = Path(__file__).resolve().parents[1] / "models" / "mobile_base.xml"

# Stability check
def check_rest(model, data):
    """Check the final second of a five-second gravity/contact simulation."""
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    heights, tilts, speeds, contacts = [], [], [], []
    for _ in range(round(5.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
            raise RuntimeError("Simulation produced non-finite state.")
        if data.time >= 4.0:
            heights.append(data.xpos[base_id, 2])
            rotation = data.xmat[base_id].reshape(3, 3)
            tilts.append(np.degrees(np.arccos(np.clip(rotation[2, 2], -1, 1))))
            speeds.append(np.linalg.norm(data.qvel[:6]))
            contacts.append(data.ncon)
    print(f"Final base height: {heights[-1]:.4f} m (nominal: 0.1500 m)")
    print(f"Maximum tilt during final second: {max(tilts):.4f} degrees")
    print(f"Maximum base velocity norm during final second: {max(speeds):.6f}")
    print(f"Minimum contact count during final second: {min(contacts)}")
    if not (
        min(heights) > 0.145 and max(heights) < 0.155
        and max(tilts) < 1.0 and max(speeds) < 0.01
        and min(contacts) >= 4
    ):
        raise RuntimeError("Base did not meet resting stability criteria.")
    print("PASS: base remains upright, supported, and stationary after settling.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Run a headless resting stability check")
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    if args.check:
        check_rest(model, data)
        return

    from mujoco.viewer import launch_passive

    with launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.12]
        viewer.cam.distance = 1.6
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        while viewer.is_running():
            start = time.perf_counter()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, model.opt.timestep - (time.perf_counter() - start)))


if __name__ == "__main__":
    main()
