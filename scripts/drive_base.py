from pathlib import Path
import time

import mujoco
from mujoco.viewer import launch_passive

MODEL = Path(__file__).resolve().parents[1] / "models" / "mobile_base.xml"

WHEEL_RADIUS = 0.09
WHEEL_SEPARATION = 0.38

def wheel_speeds_from_twist(linear_speed, angular_speed):
    left_speed = (
        linear_speed - angular_speed * WHEEL_SEPARATION/2
    ) / WHEEL_RADIUS

    right_speed = (
        linear_speed + angular_speed * WHEEL_SEPARATION/2
    ) / WHEEL_RADIUS

    return left_speed, right_speed

def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)

    left_motor_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_motor"
    )

    right_motor_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_motor"
    )

    if left_motor_id < 0 or right_motor_id < 0:
        raise RuntimeError("Could not find both wheel actuators.")

    print("Left actuator ID:", left_motor_id)
    print("Right actuator ID:", right_motor_id)
    print("Initial commands:", data.ctrl)

    with launch_passive(model, data) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = [0.4, 0, 0.12]
            viewer.cam.distance = 2.0
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25

        previous_phase = None

        while viewer.is_running():
            step_start = time.perf_counter()

            if data.time < 1.0:
                phase = "Settling"
                linear_speed = 0.0
                angular_speed = 0.0

            elif data.time < 4.0:
                phase = "Driving Forward"
                linear_speed = 0.27
                angular_speed = 0.0

            elif data.time < 5.0:
                phase = "Stopping before turn"
                linear_speed = 0.0
                angular_speed = 0.0

            elif data.time < 8.0:
                phase = "Turning Left"
                linear_speed = 0.0
                angular_speed = 0.5

            else:
                phase = "Stopping"
                linear_speed = 0.0
                angular_speed = 0.0

            # Calculate wheel targets for current phase
            left_speed, right_speed = wheel_speeds_from_twist(
                linear_speed, angular_speed)

            if phase != previous_phase:
                print(f"{data.time:.2f}s: {phase} | "
                      f"v={linear_speed: .2f} m/s, "
                      f"yaw_rate={angular_speed: .2f} rad/s | "
                      f"wheel targets: L={left_speed: .2f}, "
                      f"R={right_speed: .2f} rad/s")
                previous_phase = phase

            data.ctrl[left_motor_id] = left_speed
            data.ctrl[right_motor_id] = right_speed

            mujoco.mj_step(model, data)
            viewer.sync()

            elapsed = time.perf_counter() - step_start
            time.sleep(max(0, model.opt.timestep - elapsed))

if __name__ == "__main__":
    main()