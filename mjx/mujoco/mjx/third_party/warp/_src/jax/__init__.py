# SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import warp


def device_to_jax(warp_device: warp.DeviceLike):
    """Return the Jax device corresponding to a Warp device.

    Only Jax devices that belong to the current process are returned. A Warp CUDA device maps to the
    local Jax GPU with the same process-local ordinal (``jax.Device.local_hardware_id``), which also holds
    in multi-process programs that use :func:`jax.distributed.initialize`.

    Returns:
        :class:`jax.Device`

    Raises:
        RuntimeError: Failed to find the corresponding Jax device in the current process.
    """
    import jax  # noqa: PLC0415

    d = warp.get_device(warp_device)

    try:
        jax_devices = jax.local_devices(backend="cuda" if d.is_cuda else "cpu")
    except RuntimeError:
        jax_devices = []

    if d.is_cuda:
        for jax_device in jax_devices:
            if jax_device.local_hardware_id == d.ordinal:
                return jax_device
    elif jax_devices:
        return jax_devices[0]

    raise RuntimeError(f"Jax device corresponding to '{warp_device}' is not available")


def device_from_jax(jax_device) -> warp._src.context.Device:
    """Return the Warp device corresponding to a Jax device.

    A Jax GPU maps to the Warp CUDA device with the same process-local ordinal
    (``jax.Device.local_hardware_id``). This differs from the global ``jax.Device.id`` in multi-process
    programs that use :func:`jax.distributed.initialize`.

    Args:
        jax_device (jax.Device): A Jax device descriptor.

    Raises:
        RuntimeError: The Jax device is neither a CPU nor GPU device, or it is a GPU that belongs to
            another process.
    """
    if jax_device.platform == "cpu":
        return warp.get_device("cpu")
    elif jax_device.platform == "gpu":
        import jax  # noqa: PLC0415

        if jax_device.process_index != jax.process_index() or jax_device.local_hardware_id is None:
            raise RuntimeError(
                f"Jax device '{jax_device}' belongs to process {jax_device.process_index} "
                f"and is not addressable from process {jax.process_index()}"
            )
        return warp.get_cuda_device(jax_device.local_hardware_id)
    else:
        raise RuntimeError(f"Unsupported Jax device platform '{jax_device.platform}'")


def get_jax_device():
    """Get the current Jax device."""
    import jax  # noqa: PLC0415

    # TODO: is there a simpler way of getting the Jax "current" device?
    # check if jax.default_device() context manager is active
    device = jax.config.jax_default_device
    # if default device is not set, use first device
    if device is None:
        device = jax.local_devices()[0]
    return device


def dtype_to_jax(warp_dtype):
    """Return the Jax dtype corresponding to a Warp dtype.

    Args:
        warp_dtype: A Warp data type that has a corresponding Jax data type.

    Raises:
        TypeError: Unable to find a corresponding Jax data type.
    """
    # initialize lookup table on first call to defer jax import
    if dtype_to_jax.type_map is None:
        import jax.numpy as jp  # noqa: PLC0415

        dtype_to_jax.type_map = {
            warp.float16: jp.float16,
            warp.bfloat16: jp.bfloat16,
            warp.float32: jp.float32,
            warp.float64: jp.float64,
            warp.int8: jp.int8,
            warp.int16: jp.int16,
            warp.int32: jp.int32,
            warp.int64: jp.int64,
            warp.uint8: jp.uint8,
            warp.uint16: jp.uint16,
            warp.uint32: jp.uint32,
            warp.uint64: jp.uint64,
            warp.bool: jp.bool_,
        }

    jax_dtype = dtype_to_jax.type_map.get(warp_dtype)
    if jax_dtype is not None:
        return jax_dtype
    else:
        raise TypeError(f"Cannot convert {warp_dtype} to a Jax type")


def dtype_from_jax(jax_dtype):
    """Return the Warp dtype corresponding to a Jax dtype.

    Raises:
        TypeError: Unable to find a corresponding Warp data type.
    """
    # initialize lookup table on first call to defer jax import
    if dtype_from_jax.type_map is None:
        import jax.numpy as jp  # noqa: PLC0415

        dtype_from_jax.type_map = {
            # Jax scalar types
            jp.float16: warp.float16,
            jp.bfloat16: warp.bfloat16,
            jp.float32: warp.float32,
            jp.float64: warp.float64,
            jp.int8: warp.int8,
            jp.int16: warp.int16,
            jp.int32: warp.int32,
            jp.int64: warp.int64,
            jp.uint8: warp.uint8,
            jp.uint16: warp.uint16,
            jp.uint32: warp.uint32,
            jp.uint64: warp.uint64,
            jp.bool_: warp.bool,
            # Jax dtype objects
            jp.dtype(jp.float16): warp.float16,
            jp.dtype(jp.bfloat16): warp.bfloat16,
            jp.dtype(jp.float32): warp.float32,
            jp.dtype(jp.float64): warp.float64,
            jp.dtype(jp.int8): warp.int8,
            jp.dtype(jp.int16): warp.int16,
            jp.dtype(jp.int32): warp.int32,
            jp.dtype(jp.int64): warp.int64,
            jp.dtype(jp.uint8): warp.uint8,
            jp.dtype(jp.uint16): warp.uint16,
            jp.dtype(jp.uint32): warp.uint32,
            jp.dtype(jp.uint64): warp.uint64,
            jp.dtype(jp.bool_): warp.bool,
        }

    wp_dtype = dtype_from_jax.type_map.get(jax_dtype)
    if wp_dtype is not None:
        return wp_dtype
    else:
        raise TypeError(f"Cannot convert {jax_dtype} to a Warp type")


# lookup tables initialized when needed
dtype_from_jax.type_map = None
dtype_to_jax.type_map = None


def to_jax(warp_array):
    """
    Convert a Warp array to a Jax array without copying the data.

    Args:
        warp_array (warp.array): The Warp array to convert.

    Returns:
        jax.Array: The converted Jax array.
    """
    import jax.dlpack  # noqa: PLC0415

    return jax.dlpack.from_dlpack(warp_array)


def from_jax(jax_array, dtype=None) -> warp.array:
    """Convert a Jax array to a Warp array without copying the data.

    Args:
        jax_array (jax.Array): The Jax array to convert.
        dtype: The target data type of the resulting Warp array. Defaults to the Jax array's data type mapped to a Warp data type.

    Returns:
        warp.array: The converted Warp array.
    """

    return warp.from_dlpack(jax_array, dtype=dtype)
