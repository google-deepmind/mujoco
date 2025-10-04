#!/usr/bin/env python3
# /// script
# requires-python = "==3.11.*"
# dependencies = [
#   "jax==0.7.2",
#   "mujoco==3.2.0",
#   "mujoco-mjx==3.2.0",
# ]
# ///
import jax
from mujoco import MjModel, mjx

def main() -> None:
    host_model = MjModel.from_xml_string(model_spec_string())
    mjx_model = mjx.put_model(host_model)  # Removed impl="warp"
    initial_sim_state = mjx.make_data(host_model)  # Removed impl="warp"

    print("== Calling my compiled function for the first time ==")
    print("Result:", ball_pos_after_5_steps(mjx_model, initial_sim_state))

    print("== Calling it again ==")
    print("Result:", ball_pos_after_5_steps(mjx_model, initial_sim_state))

@jax.jit
def ball_pos_after_5_steps(model: mjx.Model, initial_sim_state: mjx.Data) -> jax.Array:
    sim_state = initial_sim_state
    for _ in range(5):
        sim_state = mjx.step(model, sim_state)
    return sim_state.qpos[:3]

def model_spec_string() -> str:
    return """<mujoco>
        <worldbody>
            <geom type="plane" size="10 10 0.1"/>
            <body pos="0 0 3">
                <geom type="sphere" size="0.2" rgba="0 1 0 1"/>
                <joint type="free"/>
            </body>
        </worldbody>
    </mujoco>"""

if __name__ == "__main__":
    main()