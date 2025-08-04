# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import enum

import jax.numpy as jnp
import numpy as np

import warp as wp

#######################################################################
# ctypes structures and enums for XLA's FFI API:
# https://github.com/openxla/xla/blob/a1a5e62fbffa3a3b6c409d72607456cf5b353a22/xla/ffi/api/c_api.h
#######################################################################


# typedef enum {
#   XLA_FFI_Extension_Metadata = 1,
# } XLA_FFI_Extension_Type;
class XLA_FFI_Extension_Type(enum.IntEnum):
    Metadata = 1


# typedef struct XLA_FFI_Extension_Base {
#   size_t struct_size;
#   XLA_FFI_Extension_Type type;
#   struct XLA_FFI_Extension_Base* next;
# } XLA_FFI_Extension_Base;
class XLA_FFI_Extension_Base(ctypes.Structure):
    pass


XLA_FFI_Extension_Base._fields_ = [
    ("struct_size", ctypes.c_size_t),
    ("type", ctypes.c_int),  # XLA_FFI_Extension_Type
    ("next", ctypes.POINTER(XLA_FFI_Extension_Base)),
]


# typedef enum {
#   XLA_FFI_ExecutionStage_INSTANTIATE = 0,
#   XLA_FFI_ExecutionStage_PREPARE = 1,
#   XLA_FFI_ExecutionStage_INITIALIZE = 2,
#   XLA_FFI_ExecutionStage_EXECUTE = 3,
# } XLA_FFI_ExecutionStage;
class XLA_FFI_ExecutionStage(enum.IntEnum):
    INSTANTIATE = 0
    PREPARE = 1
    INITIALIZE = 2
    EXECUTE = 3


# typedef enum {
#   XLA_FFI_DataType_INVALID = 0,
#   XLA_FFI_DataType_PRED = 1,
#   XLA_FFI_DataType_S8 = 2,
#   XLA_FFI_DataType_S16 = 3,
#   XLA_FFI_DataType_S32 = 4,
#   XLA_FFI_DataType_S64 = 5,
#   XLA_FFI_DataType_U8 = 6,
#   XLA_FFI_DataType_U16 = 7,
#   XLA_FFI_DataType_U32 = 8,
#   XLA_FFI_DataType_U64 = 9,
#   XLA_FFI_DataType_F16 = 10,
#   XLA_FFI_DataType_F32 = 11,
#   XLA_FFI_DataType_F64 = 12,
#   XLA_FFI_DataType_BF16 = 16,
#   XLA_FFI_DataType_C64 = 15,
#   XLA_FFI_DataType_C128 = 18,
#   XLA_FFI_DataType_TOKEN = 17,
#   XLA_FFI_DataType_F8E5M2 = 19,
#   XLA_FFI_DataType_F8E3M4 = 29,
#   XLA_FFI_DataType_F8E4M3 = 28,
#   XLA_FFI_DataType_F8E4M3FN = 20,
#   XLA_FFI_DataType_F8E4M3B11FNUZ = 23,
#   XLA_FFI_DataType_F8E5M2FNUZ = 24,
#   XLA_FFI_DataType_F8E4M3FNUZ = 25,
#   XLA_FFI_DataType_F4E2M1FN = 32,
#   XLA_FFI_DataType_F8E8M0FNU = 33,
# } XLA_FFI_DataType;
class XLA_FFI_DataType(enum.IntEnum):
    INVALID = 0
    PRED = 1
    S8 = 2
    S16 = 3
    S32 = 4
    S64 = 5
    U8 = 6
    U16 = 7
    U32 = 8
    U64 = 9
    F16 = 10
    F32 = 11
    F64 = 12
    BF16 = 16
    C64 = 15
    C128 = 18
    TOKEN = 17
    F8E5M2 = 19
    F8E3M4 = 29
    F8E4M3 = 28
    F8E4M3FN = 20
    F8E4M3B11FNUZ = 23
    F8E5M2FNUZ = 24
    F8E4M3FNUZ = 25
    F4E2M1FN = 32
    F8E8M0FNU = 33


# struct XLA_FFI_Buffer {
#   size_t struct_size;
#   XLA_FFI_Extension_Base* extension_start;
#
#   XLA_FFI_DataType dtype;
#   void* data;
#   int64_t rank;
#   int64_t* dims;  // length == rank
# };
class XLA_FFI_Buffer(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("extension_start", ctypes.POINTER(XLA_FFI_Extension_Base)),
        ("dtype", ctypes.c_int),  # XLA_FFI_DataType
        ("data", ctypes.c_void_p),
        ("rank", ctypes.c_int64),
        ("dims", ctypes.POINTER(ctypes.c_int64)),
    )


# typedef enum {
#   XLA_FFI_ArgType_BUFFER = 1,
# } XLA_FFI_ArgType;
class XLA_FFI_ArgType(enum.IntEnum):
    BUFFER = 1


# typedef enum {
#   XLA_FFI_RetType_BUFFER = 1,
# } XLA_FFI_RetType;
class XLA_FFI_RetType(enum.IntEnum):
    BUFFER = 1


# struct XLA_FFI_Args {
#   size_t struct_size;
#   XLA_FFI_Extension_Base* extension_start;
#   int64_t size;
#   XLA_FFI_ArgType* types;  // length == size
#   void** args;             // length == size
# };
class XLA_FFI_Args(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("extension_start", ctypes.POINTER(XLA_FFI_Extension_Base)),
        ("size", ctypes.c_int64),
        ("types", ctypes.POINTER(ctypes.c_int)),  # XLA_FFI_ArgType*
        ("args", ctypes.POINTER(ctypes.c_void_p)),
    )


# struct XLA_FFI_Rets {
#   size_t struct_size;
#   XLA_FFI_Extension_Base* extension_start;
#   int64_t size;
#   XLA_FFI_RetType* types;  // length == size
#   void** rets;             // length == size
# };
class XLA_FFI_Rets(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("extension_start", ctypes.POINTER(XLA_FFI_Extension_Base)),
        ("size", ctypes.c_int64),
        ("types", ctypes.POINTER(ctypes.c_int)),  # XLA_FFI_RetType*
        ("rets", ctypes.POINTER(ctypes.c_void_p)),
    )


# typedef struct XLA_FFI_ByteSpan {
#   const char* ptr;
#   size_t len;
# } XLA_FFI_ByteSpan;
class XLA_FFI_ByteSpan(ctypes.Structure):
    _fields_ = (
        ("ptr", ctypes.POINTER(ctypes.c_char)),
        ("len", ctypes.c_size_t),
    )


# typedef struct XLA_FFI_Scalar {
#   XLA_FFI_DataType dtype;
#   void* value;
# } XLA_FFI_Scalar;
class XLA_FFI_Scalar(ctypes.Structure):
    _fields_ = (
        ("dtype", ctypes.c_int),
        ("value", ctypes.c_void_p),
    )


# typedef struct XLA_FFI_Array {
#   XLA_FFI_DataType dtype;
#   size_t size;
#   void* data;
# } XLA_FFI_Array;
class XLA_FFI_Array(ctypes.Structure):
    _fields_ = (
        ("dtype", ctypes.c_int),
        ("size", ctypes.c_size_t),
        ("data", ctypes.c_void_p),
    )


# typedef enum {
#   XLA_FFI_AttrType_ARRAY = 1,
#   XLA_FFI_AttrType_DICTIONARY = 2,
#   XLA_FFI_AttrType_SCALAR = 3,
#   XLA_FFI_AttrType_STRING = 4,
# } XLA_FFI_AttrType;
class XLA_FFI_AttrType(enum.IntEnum):
    ARRAY = 1
    DICTIONARY = 2
    SCALAR = 3
    STRING = 4


# struct XLA_FFI_Attrs {
#   size_t struct_size;
#   XLA_FFI_Extension_Base* extension_start;
#   int64_t size;
#   XLA_FFI_AttrType* types;   // length == size
#   XLA_FFI_ByteSpan** names;  // length == size
#   void** attrs;              // length == size
# };
class XLA_FFI_Attrs(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("extension_start", ctypes.POINTER(XLA_FFI_Extension_Base)),
        ("size", ctypes.c_int64),
        ("types", ctypes.POINTER(ctypes.c_int)),  # XLA_FFI_AttrType*
        ("names", ctypes.POINTER(ctypes.POINTER(XLA_FFI_ByteSpan))),
        ("attrs", ctypes.POINTER(ctypes.c_void_p)),
    )


# struct XLA_FFI_Api_Version {
#   size_t struct_size;
#   XLA_FFI_Extension_Base* extension_start;
#   int major_version;  // out
#   int minor_version;  // out
# };
class XLA_FFI_Api_Version(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("extension_start", ctypes.POINTER(XLA_FFI_Extension_Base)),
        ("major_version", ctypes.c_int),
        ("minor_version", ctypes.c_int),
    )


# enum XLA_FFI_Handler_TraitsBits {
#   // Calls to FFI handler are safe to trace into the command buffer. It means
#   // that calls to FFI handler always launch exactly the same device operations
#   // (can depend on attribute values) that can be captured and then replayed.
#   XLA_FFI_HANDLER_TRAITS_COMMAND_BUFFER_COMPATIBLE = 1u << 0,
# };
class XLA_FFI_Handler_TraitsBits(enum.IntEnum):
    COMMAND_BUFFER_COMPATIBLE = 1 << 0


# struct XLA_FFI_Metadata {
#   size_t struct_size;
#   XLA_FFI_Api_Version api_version;
#   XLA_FFI_Handler_Traits traits;
# };
class XLA_FFI_Metadata(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("api_version", XLA_FFI_Api_Version),  # XLA_FFI_Extension_Type
        ("traits", ctypes.c_uint32),  # XLA_FFI_Handler_Traits
    )


# struct XLA_FFI_Metadata_Extension {
#   XLA_FFI_Extension_Base extension_base;
#   XLA_FFI_Metadata* metadata;
# };
class XLA_FFI_Metadata_Extension(ctypes.Structure):
    _fields_ = (
        ("extension_base", XLA_FFI_Extension_Base),
        ("metadata", ctypes.POINTER(XLA_FFI_Metadata)),
    )


# typedef enum {
#   XLA_FFI_Error_Code_OK = 0,
#   XLA_FFI_Error_Code_CANCELLED = 1,
#   XLA_FFI_Error_Code_UNKNOWN = 2,
#   XLA_FFI_Error_Code_INVALID_ARGUMENT = 3,
#   XLA_FFI_Error_Code_DEADLINE_EXCEEDED = 4,
#   XLA_FFI_Error_Code_NOT_FOUND = 5,
#   XLA_FFI_Error_Code_ALREADY_EXISTS = 6,
#   XLA_FFI_Error_Code_PERMISSION_DENIED = 7,
#   XLA_FFI_Error_Code_RESOURCE_EXHAUSTED = 8,
#   XLA_FFI_Error_Code_FAILED_PRECONDITION = 9,
#   XLA_FFI_Error_Code_ABORTED = 10,
#   XLA_FFI_Error_Code_OUT_OF_RANGE = 11,
#   XLA_FFI_Error_Code_UNIMPLEMENTED = 12,
#   XLA_FFI_Error_Code_INTERNAL = 13,
#   XLA_FFI_Error_Code_UNAVAILABLE = 14,
#   XLA_FFI_Error_Code_DATA_LOSS = 15,
#   XLA_FFI_Error_Code_UNAUTHENTICATED = 16
# } XLA_FFI_Error_Code;
class XLA_FFI_Error_Code(enum.IntEnum):
    OK = 0
    CANCELLED = 1
    UNKNOWN = 2
    INVALID_ARGUMENT = 3
    DEADLINE_EXCEEDED = 4
    NOT_FOUND = 5
    ALREADY_EXISTS = 6
    PERMISSION_DENIED = 7
    RESOURCE_EXHAUSTED = 8
    FAILED_PRECONDITION = 9
    ABORTED = 10
    OUT_OF_RANGE = 11
    UNIMPLEMENTED = 12
    INTERNAL = 13
    UNAVAILABLE = 14
    DATA_LOSS = 15
    UNAUTHENTICATED = 16


# struct XLA_FFI_Error_Create_Args {
#   size_t struct_size;
#   XLA_FFI_Extension_Base* extension_start;
#   const char* message;
#   XLA_FFI_Error_Code errc;
# };
class XLA_FFI_Error_Create_Args(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("extension_start", ctypes.POINTER(XLA_FFI_Extension_Base)),
        ("message", ctypes.c_char_p),
        ("errc", ctypes.c_int),
    )  # XLA_FFI_Error_Code


XLA_FFI_Error_Create = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.POINTER(XLA_FFI_Error_Create_Args))


# struct XLA_FFI_Stream_Get_Args {
#   size_t struct_size;
#   XLA_FFI_Extension_Base* extension_start;
#   XLA_FFI_ExecutionContext* ctx;
#   void* stream;  // out
# };
class XLA_FFI_Stream_Get_Args(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("extension_start", ctypes.POINTER(XLA_FFI_Extension_Base)),
        ("ctx", ctypes.c_void_p),  # XLA_FFI_ExecutionContext*
        ("stream", ctypes.c_void_p),
    )  # // out


XLA_FFI_Stream_Get = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.POINTER(XLA_FFI_Stream_Get_Args))


# struct XLA_FFI_Api {
#   size_t struct_size;
#   XLA_FFI_Extension_Base* extension_start;
#
#   XLA_FFI_Api_Version api_version;
#   XLA_FFI_InternalApi* internal_api;
#
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_Error_Create);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_Error_GetMessage);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_Error_Destroy);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_Handler_Register);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_Stream_Get);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_TypeId_Register);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_ExecutionContext_Get);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_State_Set);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_State_Get);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_DeviceMemory_Allocate);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_DeviceMemory_Free);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_ThreadPool_Schedule);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_ThreadPool_NumThreads);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_Future_Create);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_Future_SetAvailable);
#   _XLA_FFI_API_STRUCT_FIELD(XLA_FFI_Future_SetError);
# };
class XLA_FFI_Api(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("extension_start", ctypes.POINTER(XLA_FFI_Extension_Base)),
        ("api_version", XLA_FFI_Api_Version),
        ("internal_api", ctypes.c_void_p),  # XLA_FFI_InternalApi*
        ("XLA_FFI_Error_Create", XLA_FFI_Error_Create),  # XLA_FFI_Error_Create
        ("XLA_FFI_Error_GetMessage", ctypes.c_void_p),  # XLA_FFI_Error_GetMessage
        ("XLA_FFI_Error_Destroy", ctypes.c_void_p),  # XLA_FFI_Error_Destroy
        ("XLA_FFI_Handler_Register", ctypes.c_void_p),  # XLA_FFI_Handler_Register
        ("XLA_FFI_Stream_Get", XLA_FFI_Stream_Get),  # XLA_FFI_Stream_Get
        ("XLA_FFI_TypeId_Register", ctypes.c_void_p),  # XLA_FFI_TypeId_Register
        ("XLA_FFI_ExecutionContext_Get", ctypes.c_void_p),  # XLA_FFI_ExecutionContext_Get
        ("XLA_FFI_State_Set", ctypes.c_void_p),  # XLA_FFI_State_Set
        ("XLA_FFI_State_Get", ctypes.c_void_p),  # XLA_FFI_State_Get
        ("XLA_FFI_DeviceMemory_Allocate", ctypes.c_void_p),  # XLA_FFI_DeviceMemory_Allocate
        ("XLA_FFI_DeviceMemory_Free", ctypes.c_void_p),  # XLA_FFI_DeviceMemory_Free
        ("XLA_FFI_ThreadPool_Schedule", ctypes.c_void_p),  # XLA_FFI_ThreadPool_Schedule
        ("XLA_FFI_ThreadPool_NumThreads", ctypes.c_void_p),  # XLA_FFI_ThreadPool_NumThreads
        ("XLA_FFI_Future_Create", ctypes.c_void_p),  # XLA_FFI_Future_Create
        ("XLA_FFI_Future_SetAvailable", ctypes.c_void_p),  # XLA_FFI_Future_SetAvailable
        ("XLA_FFI_Future_SetError", ctypes.c_void_p),  # XLA_FFI_Future_SetError
    )


# struct XLA_FFI_CallFrame {
#   size_t struct_size;
#   XLA_FFI_Extension_Base* extension_start;
#   const XLA_FFI_Api* api;
#   XLA_FFI_ExecutionContext* ctx;
#   XLA_FFI_ExecutionStage stage;
#   XLA_FFI_Args args;
#   XLA_FFI_Rets rets;
#   XLA_FFI_Attrs attrs;
#
#   // XLA FFI handler implementation can use `future` to signal a result of
#   // asynchronous computation to the XLA runtime. XLA runtime will keep all
#   // arguments, results and attributes alive until `future` is completed.
#   XLA_FFI_Future* future;  // out
# };
class XLA_FFI_CallFrame(ctypes.Structure):
    _fields_ = (
        ("struct_size", ctypes.c_size_t),
        ("extension_start", ctypes.POINTER(XLA_FFI_Extension_Base)),
        ("api", ctypes.POINTER(XLA_FFI_Api)),
        ("ctx", ctypes.c_void_p),  # XLA_FFI_ExecutionContext*
        ("stage", ctypes.c_int),  # XLA_FFI_ExecutionStage
        ("args", XLA_FFI_Args),
        ("rets", XLA_FFI_Rets),
        ("attrs", XLA_FFI_Attrs),
        ("future", ctypes.c_void_p),  # XLA_FFI_Future* // out
    )


_xla_data_type_to_constructor = {
    # XLA_FFI_DataType.INVALID
    XLA_FFI_DataType.PRED: jnp.bool,
    XLA_FFI_DataType.S8: jnp.int8,
    XLA_FFI_DataType.S16: jnp.int16,
    XLA_FFI_DataType.S32: jnp.int32,
    XLA_FFI_DataType.S64: jnp.int64,
    XLA_FFI_DataType.U8: jnp.uint8,
    XLA_FFI_DataType.U16: jnp.uint16,
    XLA_FFI_DataType.U32: jnp.uint32,
    XLA_FFI_DataType.U64: jnp.uint64,
    XLA_FFI_DataType.F16: jnp.float16,
    XLA_FFI_DataType.F32: jnp.float32,
    XLA_FFI_DataType.F64: jnp.float64,
    XLA_FFI_DataType.BF16: jnp.bfloat16,
    XLA_FFI_DataType.C64: jnp.complex64,
    XLA_FFI_DataType.C128: jnp.complex128,
    # XLA_FFI_DataType.TOKEN
    XLA_FFI_DataType.F8E5M2: jnp.float8_e5m2,
    XLA_FFI_DataType.F8E3M4: jnp.float8_e3m4,
    XLA_FFI_DataType.F8E4M3: jnp.float8_e4m3,
    XLA_FFI_DataType.F8E4M3FN: jnp.float8_e4m3fn,
    XLA_FFI_DataType.F8E4M3B11FNUZ: jnp.float8_e4m3b11fnuz,
    XLA_FFI_DataType.F8E5M2FNUZ: jnp.float8_e5m2fnuz,
    XLA_FFI_DataType.F8E4M3FNUZ: jnp.float8_e4m3fnuz,
    # XLA_FFI_DataType.F4E2M1FN: jnp.float4_e2m1fn.dtype,
    # XLA_FFI_DataType.F8E8M0FNU: jnp.float8_e8m0fnu.dtype,
}


########################################################################
# Helpers for translating between ctypes and python types
#######################################################################


def decode_bytespan(span: XLA_FFI_ByteSpan):
    len = span.len
    chars = ctypes.cast(span.ptr, ctypes.POINTER(ctypes.c_char * len))
    return chars.contents.value.decode("utf-8")


def decode_scalar(scalar: XLA_FFI_Scalar):
    # TODO validate if dtype supported
    dtype = jnp.dtype(_xla_data_type_to_constructor[scalar.dtype])
    bytes = ctypes.string_at(scalar.value, dtype.itemsize)
    return np.frombuffer(bytes, dtype=dtype).reshape(())


def decode_array(array: XLA_FFI_Array):
    # TODO validate if dtype supported
    dtype = jnp.dtype(_xla_data_type_to_constructor[array.dtype])
    bytes = ctypes.string_at(array.data, dtype.itemsize * array.size)
    return np.frombuffer(bytes, dtype=dtype)


def decode_attrs(attrs: XLA_FFI_Attrs):
    result = {}
    for i in range(attrs.size):
        attr_name = decode_bytespan(attrs.names[i].contents)
        attr_type = attrs.types[i]
        if attr_type == XLA_FFI_AttrType.STRING:
            bytespan = ctypes.cast(attrs.attrs[i], ctypes.POINTER(XLA_FFI_ByteSpan))
            attr_value = decode_bytespan(bytespan.contents)
        elif attr_type == XLA_FFI_AttrType.SCALAR:
            attr_value = ctypes.cast(attrs.attrs[i], ctypes.POINTER(XLA_FFI_Scalar))
            attr_value = decode_scalar(attr_value.contents)
        elif attr_type == XLA_FFI_AttrType.ARRAY:
            attr_value = ctypes.cast(attrs.attrs[i], ctypes.POINTER(XLA_FFI_Array))
            attr_value = decode_array(attr_value.contents)
        elif attr_type == XLA_FFI_AttrType.DICTIONARY:
            attr_value = ctypes.cast(attrs.attrs[i], ctypes.POINTER(XLA_FFI_Attrs))
            attr_value = decode_attrs(attr_value.contents)
        else:
            raise Exception("Unexpected attr type")
        result[attr_name] = attr_value
    return result


# error-string to XLA_FFI_Error
def create_ffi_error(api, errc, message):
    create_args = XLA_FFI_Error_Create_Args(
        ctypes.sizeof(XLA_FFI_Error_Create_Args),
        ctypes.POINTER(XLA_FFI_Extension_Base)(),
        ctypes.c_char_p(message.encode("utf-8")),
        errc,
    )
    return api.contents.XLA_FFI_Error_Create(create_args)


def create_invalid_argument_ffi_error(api, message):
    return create_ffi_error(api, XLA_FFI_Error_Code.INVALID_ARGUMENT, message)


# Extract CUDA stream from XLA_FFI_CallFrame.
def get_stream_from_callframe(call_frame):
    api = call_frame.api
    get_stream_args = XLA_FFI_Stream_Get_Args(
        ctypes.sizeof(XLA_FFI_Stream_Get_Args), ctypes.POINTER(XLA_FFI_Extension_Base)(), call_frame.ctx, None
    )
    api.contents.XLA_FFI_Stream_Get(get_stream_args)
    # TODO check result
    return get_stream_args.stream


_dtype_from_ffi = {
    XLA_FFI_DataType.S8: wp.int8,
    XLA_FFI_DataType.S16: wp.int16,
    XLA_FFI_DataType.S32: wp.int32,
    XLA_FFI_DataType.S64: wp.int64,
    XLA_FFI_DataType.U8: wp.uint8,
    XLA_FFI_DataType.U16: wp.uint16,
    XLA_FFI_DataType.U32: wp.uint32,
    XLA_FFI_DataType.U64: wp.uint64,
    XLA_FFI_DataType.F16: wp.float16,
    XLA_FFI_DataType.F32: wp.float32,
    XLA_FFI_DataType.F64: wp.float64,
}


def dtype_from_ffi(ffi_dtype):
    return _dtype_from_ffi.get(ffi_dtype)


def jax_dtype_from_ffi(ffi_dtype):
    return _xla_data_type_to_constructor.get(ffi_dtype)


# Execution context (stream, stage)
class ExecutionContext:
    stage: XLA_FFI_ExecutionStage
    stream: int

    def __init__(self, callframe: XLA_FFI_CallFrame):
        self.stage = XLA_FFI_ExecutionStage(callframe.stage)
        self.stream = get_stream_from_callframe(callframe)


class FfiBuffer:
    dtype: str
    data: int
    shape: tuple[int]

    def __init__(self, xla_buffer):
        # TODO check if valid
        self.dtype = jnp.dtype(_xla_data_type_to_constructor[xla_buffer.dtype])
        self.shape = tuple(xla_buffer.dims[i] for i in range(xla_buffer.rank))
        self.data = xla_buffer.data

    @property
    def __cuda_array_interface__(self):
        return {
            "shape": self.shape,
            "typestr": self.dtype.char,
            "data": (self.data, False),
            "version": 2,
        }
