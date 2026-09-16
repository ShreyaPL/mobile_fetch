"""Verify 16 independent free-fall worlds execute on CUDA (not a benchmark)."""

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp


def main():
    wp.init()
    if not wp.is_cuda_available():
        raise RuntimeError("CUDA is unavailable; this check requires the NVIDIA GPU.")

    worlds, steps, dt = 16, 50, 0.002
    model = mujoco.MjModel.from_xml_string('''
        <mujoco>
          <option timestep="0.002" gravity="0 0 -9.81" integrator="Euler"/>
          <worldbody>
            <body pos="0 0 1">
              <freejoint/>
              <geom type="sphere" size="0.05" mass="1"/>
            </body>
          </worldbody>
        </mujoco>
    ''')
    heights = np.linspace(1.0, 2.0, worlds, dtype=np.float32)
    with wp.ScopedDevice("cuda:0"):
        gpu_model = mjw.put_model(model)
        data = mjw.make_data(model, nworld=worlds)
        initial = np.tile(model.qpos0, (worlds, 1)).astype(np.float32)
        initial[:, 2] = heights
        data.qpos.assign(initial)
        print(f"Running {worlds} worlds on {data.qpos.device}; first run compiles kernels.", flush=True)
        for _ in range(steps):
            mjw.step(gpu_model, data)
        wp.synchronize()
        positions = data.qpos.numpy()
        velocities = data.qvel.numpy()
        times = data.time.numpy()

    # Euler integrates velocity before position in this free-fall model.
    expected_z = heights - 9.81 * dt**2 * steps * (steps + 1) / 2
    np.testing.assert_allclose(positions[:, 2], expected_z, atol=2e-5, rtol=0)
    np.testing.assert_allclose(velocities[:, 2], -9.81 * dt * steps, atol=2e-5, rtol=0)
    np.testing.assert_allclose(times, dt * steps, atol=1e-6, rtol=0)
    assert np.isfinite(positions).all() and np.isfinite(velocities).all()
    print(f"PASS: {worlds} independent CUDA worlds advanced {steps} steps ({dt * steps:.3f} s each).")
    print(f"Maximum height error: {np.max(np.abs(positions[:, 2] - expected_z)):.2e} m")
    print("This verifies batched free-fall physics, not contacts, rendering, or speedup.")


if __name__ == "__main__":
    main()
