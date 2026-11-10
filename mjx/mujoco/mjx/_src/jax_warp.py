# Copyright 2024 NVIDIA CORPORATION.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import warp as wp
import ctypes
import jax.numpy as jp
import numpy as np

jax_warp_p = None

# Holder for the custom callback to keep it alive.
cc_callback = None
registered_kernels = [None]
registered_kernel_to_id = {}

def jax_kernel(wp_kernel):
    if jax_warp_p == None:
        # Create and register the primitive
        create_jax_warp_primitive()
    if not wp_kernel in registered_kernel_to_id:
        id = len(registered_kernels)
        registered_kernels.append(wp_kernel)
        registered_kernel_to_id[wp_kernel] = id
    else:
        id = registered_kernel_to_id[wp_kernel]
    def bind(*args):
       return jax_warp_p.bind(*args, kernel=id)
    return bind

def base_type(t):
  while True:
    if hasattr(t, 'dtype'):
      t = t.dtype
    elif hasattr(t, '_type_'):
      t = t._type_
    else:
      return t

def warp_custom_callback(stream, buffers, opaque, opaque_len):
    # The descriptor is the form
    # <kernel-id>|<dimensions>|<inputs>|<outputs>
    # Example:  42|16,32|fv3,2;fm33,2;f,2|f,2;fv3,2
    [kernel_id_str, dim_str, inputs_str, outputs_str] = opaque.decode().split('|')

    # Get the kernel from the registry.
    kernel = registered_kernels[int(kernel_id_str)]

    # Parse the dimensions.
    dims = [int(d) for d in dim_str.split(',')]

    # The inputs and outputs are of the following form
    # <base-type>[<matrix-vector-type],<dimension-sizes>
    # E.g., fm33,16,32 is a 16x32 array of 3x3 float matrices.
    # The individual input/output descriptors are semicolon-separated.
    args = []
    for i, a in enumerate(inputs_str.split(';') + outputs_str.split(';')):
      dtype = None
      # Parse base type.
      if a[0] == 'f':
         dtype = wp.float32
      else:
         raise Exception(f'Unknown base type "{a[0]}"')
      # Parse vector/matrix type and skip the comma.
      if a[1] == 'v':
         dtype = wp.types.vector(length=int(a[2]), dtype = dtype)
         # Index 3 is comma, let us skip.
         assert a[3] == ','
         a = a[4:]
      elif a[1] == 'm':
         dtype = wp.types.matrix(shape=(int(a[2]), int(a[3])), dtype = dtype)
         # Index 4 is comma, let us skip.
         assert a[4] == ','
         a = a[5:]
      else:
         # Index 1 is comma, let us skip.
         assert a[1] == ','
         a = a[2:]
      # Parse the array shape.
      shape = [int(s) for s in a.split(',')]
      assert len(shape) > 0, 'Currently only arrays are supported'
      # Add the array to the arg list.
      args.append(wp.array(ptr = buffers[i], dtype=dtype, shape=shape, owner=False, copy=False))

    # Launch the kernel on the provided stream.
    stream = wp.Stream(cuda_stream=ctypes.c_void_p(stream))
    wp.launch(kernel, dims, args, stream=stream, device="cuda")

def create_jax_warp_primitive():
    from functools import reduce
    import jax
    from jax._src.interpreters import batching
    from jax.interpreters import mlir
    from jax.interpreters.mlir import ir
    from jaxlib.hlo_helpers import custom_call

    global jax_warp_p
    global cc_callback

    # Create and register the primitive.
    # TODO add default implementation that calls the kernel via warp.
    jax_warp_p = jax.core.Primitive("jax_warp")
    jax_warp_p.multiple_results = True

    # TODO Just launch the kernel directly, but make sure the argument
    # shapes are massaged the same way as below so that vmap works.
    def impl(*args):
        raise Exception('Not implementes')
    jax_warp_p.def_impl(impl)

    # Auto-batching. Make sure all the arguments are fully broadcasted
    # so that Warp is not confused about dimensions.
    def vectorized_multi_batcher(args, dims, **params):
      # Figure out the number of outputs.
      wp_kernel = registered_kernels[params['kernel']]
      output_count = len(wp_kernel.adj.args) - len(args)
      shape, dim = next((a.shape, d) for a, d in zip(args, dims)
                        if d is not None)
      size = shape[dim]
      args = [batching.bdim_at_front(a, d, size) if len(a.shape) else a
              for a, d in zip(args, dims)]
      # Create the batched primitive.
      return jax_warp_p.bind(*args, **params), [dims[0]] * output_count
    batching.primitive_batchers[jax_warp_p] = vectorized_multi_batcher

    def get_mat_vec_shape(warp_type):
      if wp.types.type_is_matrix(warp_type.dtype) or wp.types.type_is_vector(warp_type.dtype):
        return warp_type.dtype._shape_
      return []

    def strip_vecmat_dimensions(warp_arg, actual_shape):
      shape = get_mat_vec_shape(warp_arg.type)
      for i, s in enumerate(reversed(shape)):
        item = actual_shape[- i - 1]
        if s != item:
            raise Exception(f'The vector/matric shape for argument {warp_arg.label} does not match')
      return actual_shape[:len(actual_shape) - len(shape)]

    def collapse_into_leading_dimension(warp_arg, actual_shape):
      if len(actual_shape) < warp_arg.type.ndim:
          raise Exception(f'Argument {warp_arg.label} has too few non-matrix/vector dimensions')
      index_rest = len(actual_shape) - warp_arg.type.ndim + 1
      leading_size = reduce(lambda x, y: x * y,actual_shape[:index_rest])
      return [leading_size] + actual_shape[index_rest:]

    # Infer array dimensions from input type.
    def infer_dimensions(warp_arg, actual_shape):
      actual_shape = strip_vecmat_dimensions(warp_arg, actual_shape)
      return collapse_into_leading_dimension(warp_arg, actual_shape)

    # Abstract evaluation.
    def jax_warp_abstract(*args, kernel=None):
        wp_kernel = registered_kernels[kernel]
        dtype = jax.dtypes.canonicalize_dtype(args[0].dtype)
        # All the extra arguments to the warp kernel are outputs.
        outputs = [ o.type for o in wp_kernel.adj.args[len(args):] ]
        # Let's just use the first input dimension to infer the output's dimensions.
        dims = strip_vecmat_dimensions(wp_kernel.adj.args[0], list(args[0].shape))
        return [ jax.core.ShapedArray(list(dims) + list(get_mat_vec_shape(o)), dtype) for o in outputs ]
    jax_warp_p.def_abstract_eval(jax_warp_abstract)

    # Lowering to MLIR.

    # Create python-land custom call target.
    CCALLFUNC = ctypes.CFUNCTYPE(ctypes.c_voidp, ctypes.c_void_p,
                                 ctypes.POINTER(ctypes.c_void_p),
                                 ctypes.c_char_p, ctypes.c_size_t)
    cc_callback = CCALLFUNC(warp_custom_callback)
    ccall_address = ctypes.cast(cc_callback, ctypes.c_void_p)

    # Put the custom call into a capsule, as required by XLA.
    PyCapsule_Destructor = ctypes.CFUNCTYPE(None, ctypes.py_object)
    PyCapsule_New = ctypes.pythonapi.PyCapsule_New
    PyCapsule_New.restype = ctypes.py_object
    PyCapsule_New.argtypes = (ctypes.c_void_p, ctypes.c_char_p, PyCapsule_Destructor)
    capsule = PyCapsule_New(ccall_address.value,
                            b"xla._CUSTOM_CALL_TARGET", PyCapsule_Destructor(0))

    # Register the callback in XLA.
    jax.lib.xla_client.register_custom_call_target("warp_call", capsule, platform="gpu")

    def default_layout(shape):
        return range(len(shape) - 1, -1, -1)

    def warp_call_lowering(ctx, *args, kernel=None):
        if not kernel:
            raise Exception('Unknown kernel ' + str(kernel))
        wp_kernel = registered_kernels[kernel]

        # TODO This may not be necessary, but it is perhaps better not to be
        # mucking with kernel loading while already running the workload.
        module = wp_kernel.module
        device = "cuda"
        if not module.load(device):
            raise Exception("Could not load kernel on device")

        # Infer dimensions from the first input.
        warp_arg0 = wp_kernel.adj.args[0]
        actual_shape0 = ir.RankedTensorType(args[0].type).shape
        dims = strip_vecmat_dimensions(warp_arg0, actual_shape0)
        warp_dims = collapse_into_leading_dimension(warp_arg0, dims)

        # Figure out the types and sizes and create the descriptor for the inputs.
        i = 0
        input_descriptors = []
        operand_layouts = []
        for actual, warg in zip(args, wp_kernel.adj.args):
          # Check supported cases.
          wtype = warg.type
          if not wp.types.is_array(wtype):
             raise Exception('Only arrays are supported')
          if base_type(wtype) == 'f':
             if str(ir.RankedTensorType(actual.type).element_type) != 'f32':
                raise Exception(f'Unexpected base type for {warg.label}')
          else:
             raise Exception(f'Currently only float32 is supported')

          # Add the base type to the descriptor.
          desc = base_type(warg.type)
          # Add vector/matrix types.
          shape = []
          if wp.types.type_is_matrix(wtype.dtype):
             desc += 'm'
          if wp.types.type_is_vector(wtype.dtype):
             desc += 'v'
          # Get matrix/vector shapes and check that they fit.
          if wp.types.type_is_matrix(wtype.dtype) or wp.types.type_is_vector(wtype.dtype):
            shape = wtype.dtype._shape_
          desc += ''.join([str(s) for s in shape])
          # Infer array dimension (by removing the vector/matrix dimensions and
          # collapsing the initial dimensions).
          array_shape = infer_dimensions(warg, ir.RankedTensorType(actual.type).shape)
          desc += ',' + ','.join([str(s) for s in array_shape])
          input_descriptors.append(desc)
          operand_layouts.append(default_layout(ir.RankedTensorType(actual.type).shape))
          i += 1

        # Infer dimensions from the first input.
        output_descriptors = []
        result_types = []
        result_layouts = []
        for warg in wp_kernel.adj.args[len(args):]:
          wtype = warg.type
          # Add base type to descriptor.
          desc = base_type(warg.type)
          # Add vector/matrix types to descriptor if needed.
          shape = []
          if wp.types.type_is_matrix(wtype.dtype):
             desc += 'm'
          if wp.types.type_is_vector(wtype.dtype):
             desc += 'v'
          # Get matrix/vector shapes and check that they fit.
          if wp.types.type_is_matrix(wtype.dtype) or wp.types.type_is_vector(wtype.dtype):
            shape = wtype.dtype._shape_
          desc += ''.join([str(s) for s in shape])
          # Add the dimensions (harvested from the first input).
          desc += ',' + ','.join([str(s) for s in warp_dims])
          output_descriptors.append(desc)
          result_shape = list(dims) + list(shape)
          result_types.append((ir.RankedTensorType.get(result_shape, ir.F32Type.get())))
          result_layouts.append(default_layout(result_shape))

        # Build the full descriptor.
        descriptor = str(kernel) + "|" + ','.join([str(d) for d in warp_dims]) + "|"
        descriptor += ';'.join(input_descriptors) + '|'
        descriptor += ';'.join(output_descriptors)
        print("Descriptor for custom call: ", descriptor)

        out = custom_call(
            b"warp_call",
            result_types=result_types,
            operands=args,
            backend_config=descriptor.encode('utf-8'),
            operand_layouts=operand_layouts,
            result_layouts=result_layouts,
        ).results
        return out

    mlir.register_lowering(
        jax_warp_p,
        warp_call_lowering,
        platform="gpu",
    )
