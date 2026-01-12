import jax
import jax.numpy as jnp
import warp as wp
import numpy as np
from mujoco.mjx.third_party.warp._src.jax_experimental import ffi

wp.init()

# Define a simple Warp kernel
@wp.kernel
def simple_kernel(x: wp.array(dtype=float), y: wp.array(dtype=float)):
    tid = wp.tid()
    y[tid] = x[tid] * 2.0

# Wrap it for JAX
# We use a pattern similar to how MJX wraps collisions
def call_kernel(x):
    # output same shape as input
    out_dim = {'y': x.shape}
    
    # Define the FFI callable
    kernel_ffi = ffi.jax_kernel(
        simple_kernel,
        num_outputs=1,
        launch_dims=x.shape[0], # 1D launch
        output_dims=x.shape,
    )
    
    return kernel_ffi(x)

# pmap entry point
def pmap_step(x):
    return call_kernel(x)

def main():
    print("Initializing...")
    key = jax.random.PRNGKey(0)
    
    # Number of devices to simulate pmap
    n_devices = jax.local_device_count()
    print(f"Devices: {n_devices}")
    
    # Create batched input for pmap
    batch_size = n_devices
    data_size = 100000
    x = jax.random.normal(key, (batch_size, data_size))
    
    print("Compiling pmap...")
    # Compile with donate_argnums
    pmapped_fn = jax.pmap(pmap_step, donate_argnums=(0,))
    
    print("Running loop...")
    for i in range(1000):
        if i % 100 == 0:
            print(f"Step {i}")
        # Pass x, get x_new. Since we donate x, jax can reuse it.
        # But wait, pmap outputs are sharded arrays.
        # We need to block_until_ready explicitly or just usage will trigger it.
        x = pmapped_fn(x)
        x.block_until_ready()

    print("Finished successfully.")

if __name__ == "__main__":
    main()
