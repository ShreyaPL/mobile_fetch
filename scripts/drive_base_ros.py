import time

import mujoco
from mujoco.viewer import launch_passive
import rclpy
from rclpy.executors import ExternalShutdownException

from drive_base import MODEL, wheel_speeds_from_twist
from mobile_fetch_control.velocity_receiver import VelocityReceiver

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

    left_joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_joint"
    )
    right_joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_joint"
    )

    if left_joint_id < 0 or right_joint_id < 0:
        raise RuntimeError("Could not find both wheel joints.")

    left_position_index = model.jnt_qposadr[left_joint_id]
    right_position_index = model.jnt_qposadr[right_joint_id]

    left_velocity_index = model.jnt_dofadr[left_joint_id]
    right_velocity_index = model.jnt_dofadr[right_joint_id]

    rclpy.init()
    node = VelocityReceiver()

    try:
        joint_publish_period = 0.02
        next_joint_publish_time = 0.0
        with launch_passive(model, data) as viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = [0.4, 0, 0.12]
                viewer.cam.distance = 2.0
                viewer.cam.azimuth = 135
                viewer.cam.elevation = -25

            node.get_logger().info(
                "Simulation ready. Waiting for /cmd_vel."
            )

            while rclpy.ok() and viewer.is_running():
                step_start = time.perf_counter()

                #1. Let ROS2 process a waiting message
                rclpy.spin_once(node, timeout_sec=0.0)

                #2. Read the command, or zeros if it has expired.
                linear_speed, angular_speed = node.get_velocity_command()

                #3. Convert robot velocity to wheel targets
                left_speed, right_speed = wheel_speeds_from_twist(linear_speed, angular_speed)

                #4. Apply commands and advance physics
                data.ctrl[left_motor_id] = left_speed
                data.ctrl[right_motor_id] = right_speed
                mujoco.mj_step(model, data)
                node.publish_sim_time(data.time)

                if data.time >=next_joint_publish_time:
                    positions = [
                        data.qpos[left_position_index],
                        data.qpos[right_position_index],
                    ]

                    velocities = [
                        data.qvel[left_velocity_index],
                        data.qvel[right_velocity_index],
                    ]

                    node.publish_joint_states(
                        data.time,
                        positions,
                        velocities
                    )

                    next_joint_publish_time = (
                        data.time + joint_publish_period
                    )
                
                #5. Update display and pace the loop
                viewer.sync()
                elapsed = time.perf_counter() - step_start
                time.sleep(max(0.0, model.opt.timestep - elapsed))

    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()