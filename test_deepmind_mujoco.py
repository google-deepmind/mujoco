import sys
import os

# Weka njia ya python package ya mujoco kwenye sys.path
mujoco_python_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "python"))
if mujoco_python_path not in sys.path:
    sys.path.insert(0, mujoco_python_path)

import time
import gc
import psutil
import mujoco

def get_memory_mb():
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 * 1024)

# Complex XML Physics Model
XML_MODEL = """
<mujoco model="pendulum_chain">
  <compiler angle="degree"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="plane" size="2 2 0.1"/>
    <body pos="0 0 2">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0  0 0 -0.5" size="0.05"/>
      <body pos="0 0 -0.5">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0  0 0 -0.5" size="0.04"/>
        <body pos="0 0 -0.5">
          <joint name="j3" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.08" rgba="1 0 0 1"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

def run_simulation(steps=2_000_000, use_atsonic=False):
    model = mujoco.MjModel.from_xml_string(XML_MODEL)
    data = mujoco.MjData(model)
    
    mode_str = "AtSonic UNGC Engine" if use_atsonic else "Standard Python GC"
    print(f"\n==================================================")
    print(f" Starting MuJoCo Real Physics: {steps:,} Steps")
    print(f" Mode: {mode_str}")
    print(f"==================================================")

    if use_atsonic:
        success = mujoco.enable_atsonic_ungc(capacity=steps)
        print(f" AtSonic Engine Status Active: {mujoco.is_atsonic_active()}")
    else:
        mujoco.disable_atsonic_ungc()
        print(f" Python GC Status Enabled: {gc.isenabled()}")

    mem_start = get_memory_mb()
    t_start = time.perf_counter()

    # Core MuJoCo Physics Computation Loop
    for step in range(steps):
        mujoco.mj_step(model, data)

    t_end = time.perf_counter()
    mem_end = get_memory_mb()

    elapsed = t_end - t_start
    fps = steps / elapsed
    avg_latency_us = (elapsed / steps) * 1e6

    print(f"\n Performance Metrics:")
    print(f" Total Elapsed Time : {elapsed:.4f} s")
    print(f" Physics Throughput : {fps:,.2f} steps/sec (FPS)")
    print(f" Avg Step Latency   : {avg_latency_us:.4f} µs")
    print(f" Initial Memory     : {mem_start:.2f} MB")
    print(f" Final Memory       : {mem_end:.2f} MB")
    print(f" Memory Overhead    : {mem_end - mem_start:.2f} MB")

if __name__ == "__main__":
    # Test 1: Standard Execution
    run_simulation(steps=2_000_000, use_atsonic=False)
    
    # Test 2: AtSonic Engine Integrated Execution
    run_simulation(steps=2_000_000, use_atsonic=True)
