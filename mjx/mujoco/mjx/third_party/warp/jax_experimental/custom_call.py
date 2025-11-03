# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ctypes

import warp as wp
from warp.context import type_str
from warp.jax import get_jax_device
from warp.types import array_t, launch_bounds_t, strides_from_shape
from warp.utils import warn

_jax_warp_p = None

# Holder for the custom callback to keep it alive.
_cc_callback = None
_registered_kernels = [None]
_registered_kernel_to_id = {}


def jax_kernel(kernel, launch_dims=None, quiet=False):
    """Create a Jax primitive from a Warp kernel.

    NOTE: This is an experimental feature under development.

    Args:
        kernel: The Warp kernel to be wrapped.
        launch_dims: Optional. Specify the kernel launch dimensions. If None,
                     dimensions are inferred from the shape of the first argument.
                     This option when set will specify the output dimensions.
        quiet: Optional. If True, suppress deprecation warnings with newer JAX versions.

    Limitations:
        - All kernel arguments must be contiguous arrays.
        - Input arguments are followed by output arguments in the Warp kernel definition.
        - There must be at least one input argument and at least one output argument.
        - Only the CUDA backend is supported.
    """

    import jax

    # check if JAX version supports this
    if jax.__version_info__ < (0, 4, 25) or jax.__version_info__ >= (0, 8, 0):
        msg = (
            "This version of jax_kernel() requires JAX version 0.4.25 - 0.7.x, "
            f"but installed JAX version is {jax.__version_info__}."
        )
        if jax.__version_info__ >= (0, 8, 0):
            msg += " Please use warp.jax_experimental.ffi.jax_kernel instead."
        raise RuntimeError(msg)

    # deprecation warning
    if jax.__version_info__ >= (0, 5, 0) and not quiet:
        warn(
            "This version of jax_kernel() is deprecated and will not be supported with newer JAX versions. "
            "Please use the newer FFI version instead (warp.jax_experimental.ffi.jax_kernel). "
            "In Warp release 1.10, the FFI version will become the default implementation of jax_kernel().",
            DeprecationWarning,
        )

    if _jax_warp_p is None:
        # Create and register the primitive
        _create_jax_warp_primitive()
    if kernel not in _registered_kernel_to_id:
        id = len(_registered_kernels)
        _registered_kernels.append(kernel)
        _registered_kernel_to_id[kernel] = id
    else:
        id = _registered_kernel_to_id[kernel]

    def bind(*args):
        return _jax_warp_p.bind(*args, kernel=id, launch_dims=launch_dims)

    return bind


def _warp_custom_callback(stream, buffers, opaque, opaque_len):
    # The descriptor is the form
    # <kernel-id>|<launch-dims>|<arg-dims-list>
    # Example:  42|16,32|16,32;100;16,32
    kernel_id_str, dim_str, args_str = opaque.decode().split("|")

    # Get the kernel from the registry.
    kernel_id = int(kernel_id_str)
    kernel = _registered_kernels[kernel_id]

    # Parse launch dimensions.
    dims = [int(d) for d in dim_str.split(",")]
    bounds = launch_bounds_t(dims)

    # Parse arguments.
    arg_strings = args_str.split(";")
    num_args = len(arg_strings)
    assert num_args == len(kernel.adj.args), "Incorrect number of arguments"

    # First param is the launch bounds.
    kernel_params = (ctypes.c_void_p * (1 + num_args))()
    kernel_params[0] = ctypes.addressof(bounds)

    # Parse array descriptors.
    args = []
    for i in range(num_args):
        dtype = kernel.adj.args[i].type.dtype
        shape = [int(d) for d in arg_strings[i].split(",")]
        strides = strides_from_shape(shape, dtype)

        arr = array_t(buffers[i], 0, len(shape), shape, strides)
        args.append(arr)  # keep a reference
        arg_ptr = ctypes.addressof(arr)

        kernel_params[i + 1] = arg_ptr

    # Get current device.
    device = wp.device_from_jax(get_jax_device())

    # Get kernel hooks.
    # Note: module was loaded during jit lowering.
    hooks = kernel.module.get_kernel_hooks(kernel, device)
    assert hooks.forward, "Failed to find kernel entry point"

    # Launch the kernel.
    wp.context.runtime.core.wp_cuda_launch_kernel(
        device.context, hooks.forward, bounds.size, 0, 256, hooks.forward_smem_bytes, kernel_params, stream
    )


def _create_jax_warp_primitive():
    from functools import reduce

    import jax
    from jax._src.interpreters import batching
    from jax.interpreters import mlir
    from jax.interpreters.mlir import ir
    from jax.google.hlo_helpers import custom_call

    global _jax_warp_p
    global _cc_callback

    # Create and register the primitive.
    # TODO add default implementation that calls the kernel via warp.
    try:
        # newer JAX versions
        import jax.extend

        _jax_warp_p = jax.extend.core.Primitive("jax_warp")
    except (ImportError, AttributeError):
        # older JAX versions
        _jax_warp_p = jax.core.Primitive("jax_warp")
    _jax_warp_p.multiple_results = True

    # TODO Just launch the kernel directly, but make sure the argument
    # shapes are massaged the same way as below so that vmap works.
    def impl(*args):
        raise Exception("Not implemented")

    _jax_warp_p.def_impl(impl)

    # Auto-batching. Make sure all the arguments are fully broadcasted
    # so that Warp is not confused about dimensions.
    def vectorized_multi_batcher(args, dims, **params):
        # Figure out the number of outputs.
        wp_kernel = _registered_kernels[params["kernel"]]
        output_count = len(wp_kernel.adj.args) - len(args)
        shape, dim = next((a.shape, d) for a, d in zip(args, dims) if d is not None)
        size = shape[dim]
        args = [batching.bdim_at_front(a, d, size) if len(a.shape) else a for a, d in zip(args, dims)]
        # Create the batched primitive.
        return _jax_warp_p.bind(*args, **params), [dims[0]] * output_count

    batching.primitive_batchers[_jax_warp_p] = vectorized_multi_batcher

    def get_vecmat_shape(warp_type):
        if hasattr(warp_type.dtype, "_shape_"):
            return warp_type.dtype._shape_
        return []

    def strip_vecmat_dimensions(warp_arg, actual_shape):
        shape = get_vecmat_shape(warp_arg.type)
        for i, s in enumerate(reversed(shape)):
            item = actual_shape[-i - 1]
            if s != item:
                raise Exception(f"The vector/matrix shape for argument {warp_arg.label} does not match")
        return actual_shape[: len(actual_shape) - len(shape)]

    def collapse_into_leading_dimension(warp_arg, actual_shape):
        if len(actual_shape) < warp_arg.type.ndim:
            raise Exception(f"Argument {warp_arg.label} has too few non-matrix/vector dimensions")
        index_rest = len(actual_shape) - warp_arg.type.ndim + 1
        leading_size = reduce(lambda x, y: x * y, actual_shape[:index_rest])
        return [leading_size] + actual_shape[index_rest:]

    # Infer array dimensions from input type.
    def infer_dimensions(warp_arg, actual_shape):
        actual_shape = strip_vecmat_dimensions(warp_arg, actual_shape)
        return collapse_into_leading_dimension(warp_arg, actual_shape)

    def base_type_to_jax(warp_dtype):
        if hasattr(warp_dtype, "_wp_scalar_type_"):
            return wp.dtype_to_jax(warp_dtype._wp_scalar_type_)
        return wp.dtype_to_jax(warp_dtype)

    def base_type_to_jax_ir(warp_dtype):
        warp_to_jax_dict = {
            wp.float16: ir.F16Type.get(),
            wp.float32: ir.F32Type.get(),
            wp.float64: ir.F64Type.get(),
            wp.int8: ir.IntegerType.get_signless(8),
            wp.int16: ir.IntegerType.get_signless(16),
            wp.int32: ir.IntegerType.get_signless(32),
            wp.int64: ir.IntegerType.get_signless(64),
            wp.uint8: ir.IntegerType.get_unsigned(8),
            wp.uint16: ir.IntegerType.get_unsigned(16),
            wp.uint32: ir.IntegerType.get_unsigned(32),
            wp.uint64: ir.IntegerType.get_unsigned(64),
        }
        if hasattr(warp_dtype, "_wp_scalar_type_"):
            warp_dtype = warp_dtype._wp_scalar_type_
        jax_dtype = warp_to_jax_dict.get(warp_dtype)
        if jax_dtype is None:
            raise TypeError(f"Invalid or unsupported data type: {warp_dtype}")
        return jax_dtype

    def base_type_is_compatible(warp_type, jax_ir_type):
        jax_ir_to_warp = {
            "f16": wp.float16,
            "f32": wp.float32,
            "f64": wp.float64,
            "i8": wp.int8,
            "i16": wp.int16,
            "i32": wp.int32,
            "i64": wp.int64,
            "ui8": wp.uint8,
            "ui16": wp.uint16,
            "ui32": wp.uint32,
            "ui64": wp.uint64,
        }
        expected_warp_type = jax_ir_to_warp.get(str(jax_ir_type))
        if expected_warp_type is not None:
            if hasattr(warp_type, "_wp_scalar_type_"):
                return warp_type._wp_scalar_type_ == expected_warp_type
            else:
                return warp_type == expected_warp_type
        else:
            raise TypeError(f"Invalid or unsupported data type: {jax_ir_type}")

    # Abstract evaluation.
    def jax_warp_abstract(*args, kernel=None, launch_dims=None):
        wp_kernel = _registered_kernels[kernel]
        # All the extra arguments to the warp kernel are outputs.
        warp_outputs = [o.type for o in wp_kernel.adj.args[len(args) :]]

        if launch_dims is None:
            # Use the first input dimension to infer the output's dimensions if launch_dims is not provided
            dims = strip_vecmat_dimensions(wp_kernel.adj.args[0], list(args[0].shape))
        else:
            dims = launch_dims

        jax_outputs = []
        for o in warp_outputs:
            shape = list(dims) + list(get_vecmat_shape(o))
            dtype = base_type_to_jax(o.dtype)
            jax_outputs.append(jax.core.ShapedArray(shape, dtype))
        return jax_outputs

    _jax_warp_p.def_abstract_eval(jax_warp_abstract)

    # Lowering to MLIR.

    # Create python-land custom call target.
    CCALLFUNC = ctypes.CFUNCTYPE(
        ctypes.c_voidp, ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_size_t
    )
    _cc_callback = CCALLFUNC(_warp_custom_callback)
    ccall_address = ctypes.cast(_cc_callback, ctypes.c_void_p)

    # Put the custom call into a capsule, as required by XLA.
    PyCapsule_Destructor = ctypes.CFUNCTYPE(None, ctypes.py_object)
    PyCapsule_New = ctypes.pythonapi.PyCapsule_New
    PyCapsule_New.restype = ctypes.py_object
    PyCapsule_New.argtypes = (ctypes.c_void_p, ctypes.c_char_p, PyCapsule_Destructor)
    capsule = PyCapsule_New(ccall_address.value, b"xla._CUSTOM_CALL_TARGET", PyCapsule_Destructor(0))

    # Register the callback in XLA.
    try:
        # newer JAX versions
        jax.ffi.register_ffi_target("warp_call", capsule, platform="gpu", api_version=0)
    except AttributeError:
        # older JAX versions
        jax.lib.xla_client.register_custom_call_target("warp_call", capsule, platform="gpu")

    def default_layout(shape):
        return range(len(shape) - 1, -1, -1)

    def warp_call_lowering(ctx, *args, kernel=None, launch_dims=None):
        if not kernel:
            raise Exception("Unknown kernel id " + str(kernel))
        wp_kernel = _registered_kernels[kernel]

        # TODO This may not be necessary, but it is perhaps better not to be
        # mucking with kernel loading while already running the workload.
        module = wp_kernel.module
        device = wp.device_from_jax(get_jax_device())
        if not module.load(device):
            raise Exception("Could not load kernel on device")

        if launch_dims is None:
            # Infer dimensions from the first input.
            warp_arg0 = wp_kernel.adj.args[0]
            actual_shape0 = ir.RankedTensorType(args[0].type).shape
            dims = strip_vecmat_dimensions(warp_arg0, actual_shape0)
            warp_dims = collapse_into_leading_dimension(warp_arg0, dims)
        else:
            dims = launch_dims
            warp_dims = launch_dims
        # Figure out the types and shapes of the input arrays.
        arg_strings = []
        operand_layouts = []
        for actual, warg in zip(args, wp_kernel.adj.args):
            wtype = warg.type
            rtt = ir.RankedTensorType(actual.type)

            if not isinstance(wtype, wp.array):
                raise Exception("Only contiguous arrays are supported for Jax kernel arguments")

            if not base_type_is_compatible(wtype.dtype, rtt.element_type):
                raise TypeError(
                    f"Incompatible data type for argument '{warg.label}', expected {type_str(wtype.dtype)}, got {rtt.element_type}"
                )

            # Infer array dimension (by removing the vector/matrix dimensions and
            # collapsing the initial dimensions).
            shape = infer_dimensions(warg, rtt.shape)

            if len(shape) != wtype.ndim:
                raise TypeError(f"Incompatible array dimensionality for argument '{warg.label}'")

            arg_strings.append(",".join([str(d) for d in shape]))
            operand_layouts.append(default_layout(rtt.shape))

        # Figure out the types and shapes of the output arrays.
        result_types = []
        result_layouts = []
        for warg in wp_kernel.adj.args[len(args) :]:
            wtype = warg.type

            if not isinstance(wtype, wp.array):
                raise Exception("Only contiguous arrays are supported for Jax kernel arguments")

            # Infer dimensions from the first input.
            arg_strings.append(",".join([str(d) for d in warp_dims]))

            result_shape = list(dims) + list(get_vecmat_shape(wtype))
            result_types.append(ir.RankedTensorType.get(result_shape, base_type_to_jax_ir(wtype.dtype)))
            result_layouts.append(default_layout(result_shape))

        # Build opaque descriptor for callback.
        shape_str = ",".join([str(d) for d in warp_dims])
        args_str = ";".join(arg_strings)
        descriptor = f"{kernel}|{shape_str}|{args_str}"

        out = custom_call(
            b"warp_call",
            result_types=result_types,
            operands=args,
            backend_config=descriptor.encode("utf-8"),
            operand_layouts=operand_layouts,
            result_layouts=result_layouts,
        ).results
        return out

    mlir.register_lowering(
        _jax_warp_p,
        warp_call_lowering,
        platform="gpu",
    )
