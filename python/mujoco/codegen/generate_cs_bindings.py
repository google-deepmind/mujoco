# Copyright 2026 DeepMind Technologies Limited
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
"""Generates C# bindings for MuJoCo from introspect modules."""

import re
import sys
import ctypes

from absl import app
from absl import flags
from introspect import ast_nodes
from introspect import enums as introspect_enums
from introspect import functions as introspect_functions
from introspect import structs as introspect_structs

_HEADER_FILES = flags.DEFINE_string(
    'header_file_list', '', 'Space-separated list of MuJoCo header files.'
)

_BOILERPLATE_IMPORTS = [
    'using System;',
    'using System.Runtime.InteropServices;',
    'using System.Text;',
    'using System.Collections.Generic;',
    'using SizeDict = System.Collections.Generic.Dictionary<string, string[]>;',
    (
        'using NestedDictionary = System.Collections.Generic'
        '.Dictionary<string, System.Collections.Generic'
        '.Dictionary<string, string[]>>;'
    ),
]

_CS_RESERVED_KEYWORDS = {
    'abstract',
    'as',
    'base',
    'bool',
    'break',
    'byte',
    'case',
    'catch',
    'char',
    'checked',
    'class',
    'const',
    'continue',
    'decimal',
    'default',
    'delegate',
    'do',
    'double',
    'else',
    'enum',
    'event',
    'explicit',
    'extern',
    'false',
    'finally',
    'fixed',
    'float',
    'for',
    'foreach',
    'goto',
    'if',
    'implicit',
    'in',
    'int',
    'interface',
    'internal',
    'is',
    'lock',
    'long',
    'namespace',
    'new',
    'null',
    'object',
    'operator',
    'out',
    'override',
    'params',
    'private',
    'protected',
    'public',
    'readonly',
    'ref',
    'return',
    'sbyte',
    'sealed',
    'short',
    'sizeof',
    'stackalloc',
    'static',
    'string',
    'struct',
    'switch',
    'this',
    'throw',
    'true',
    'try',
    'typeof',
    'uint',
    'ulong',
    'unchecked',
    'unsafe',
    'ushort',
    'using',
    'virtual',
    'void',
    'volatile',
    'while',
}

C_TO_CS_TYPE = {
    'int': 'int',
    'unsigned int': 'uint',
    'float': 'float',
    'double': 'double',
    'char': 'char',
    'unsigned char': 'byte',
    'size_t': 'UIntPtr',
    'int64_t': 'Int64',
    'uint64_t': 'UInt64',
    'uintptr_t': 'UIntPtr',
    'void': 'void',
    'mjtNum': 'double',
    'mjtByte': 'byte',
    'mjtSize': 'UInt64' if ctypes.sizeof(ctypes.c_size_t) == 8 else 'UInt32',
}

PYTHON_TYPE_TO_CS_TYPE = {
    'float': 'double',
}

PYTHON_VALUE_TO_CS_VALUE = {
    True: 'true',
    False: 'false',
}

_BLITTABLE_TYPES = {
    'Byte',
    'byte',
    'SByte',
    'sbyte',
    'Int16',
    'short',
    'UInt16',
    'ushort',
    'Int32',
    'int',
    'UInt32',
    'uint',
    'Int64',
    'long',
    'UInt64',
    'ulong',
    'IntPtr',
    'UIntPtr',
    'Single',
    'float',
    'double',
    'void',
    'char',
}

_ALLOWED_FIXED_ARRAYS = {
    'bool',
    'byte',
    'short',
    'int',
    'long',
    'char',
    'sbyte',
    'ushort',
    'uint',
    'ulong',
    'float',
    'double',
}

_FUNCTION_POINTER_TYPES = {
    'mjfItemEnable',
}

_STRUCT_NAME_OVERRIDES = {}

_STUB_STRUCTS = [
    '_mjVFS',
    'mjuiItemSingle_',
    'mjuiItemMulti_',
    'mjuiItemSlider_',
    'mjuiItemEdit_',
]

_OPAQUE_STRUCTS = [
    'mjVFS',
    'mjSDF',
    'mjTask',
    'mjThreadPool',
    'mjpDecoder',
    'mjpResourceProvider',
    'mjsElement',
    'mjString',
    'mjStringVec',
    'mjIntVec',
    'mjIntVecVec',
    'mjFloatVec',
    'mjFloatVecVec',
    'mjDoubleVec',
    'mjByteVec',
    'mjSpec',
    'mjsOrientation',
    'mjsPlugin',
    'mjsBody',
    'mjsFrame',
    'mjsJoint',
    'mjsGeom',
    'mjsSite',
    'mjsCamera',
    'mjsLight',
    'mjsFlex',
    'mjsMesh',
    'mjsHField',
    'mjsSkin',
    'mjsTexture',
    'mjsMaterial',
    'mjsPair',
    'mjsExclude',
    'mjsEquality',
    'mjsTendon',
    'mjsWrap',
    'mjsActuator',
    'mjsSensor',
    'mjsNumeric',
    'mjsText',
    'mjsTuple',
    'mjsKey',
    'mjsDefault',
]

_OMITTED_FUNCTIONS = [
    'mj_loadAllPluginLibraries',
    'mjc_distance',
    'mjc_getSDF',
    'mjc_gradient',
    'mjp_',
    'mju_decodeResource',
    'mju_defaultTask',
    'mju_task',
    'mju_thread',
    'mju_openResource',
    'mju_closeResource',
    'mju_readResource',
    'mju_getResourceDir',
    'mju_isModifiedResource',
    'mj_compile',
    'mj_copyBack',
    'mj_copySpec',
    'mj_deleteSpec',
    'mj_makeSpec',
    'mj_parse',
    'mj_parseXML',
    'mj_parseXMLString',
    'mj_recompile',
    'mj_saveXML',
    'mj_saveXMLString',
    'mj_mountVFS',
    'mj_unmountVFS',
    'mjs_',
]


def _mangle_name(name):
  while name in _CS_RESERVED_KEYWORDS:
    name += '_'
  return name


def _strip_struct_prefix(name):
  if name.startswith('struct '):
    return name[len('struct ') :]
  return name


def _resolve_type_name(raw_name):
  name = _strip_struct_prefix(raw_name)
  if name in _FUNCTION_POINTER_TYPES:
    return 'IntPtr'
  if name in C_TO_CS_TYPE:
    return C_TO_CS_TYPE[name]
  if name in _STRUCT_NAME_OVERRIDES:
    return _STRUCT_NAME_OVERRIDES[name]
  if name.endswith('_'):
    return name
  return name + '_'


def _is_opaque_type(raw_name):
  name = _strip_struct_prefix(raw_name)
  bare = name.rstrip('_')
  return bare in _OPAQUE_STRUCTS


def _resolve_cs_type(type_node):
  if isinstance(type_node, ast_nodes.ValueType):
    return _resolve_type_name(type_node.name)
  elif isinstance(type_node, ast_nodes.PointerType):
    if isinstance(
        type_node.inner_type, ast_nodes.ValueType
    ) and _is_opaque_type(type_node.inner_type.name):
      return 'void*'
    inner_cs = _resolve_cs_type(type_node.inner_type)
    return inner_cs + '*'
  elif isinstance(type_node, ast_nodes.ArrayType):
    inner_cs = _resolve_cs_type(type_node.inner_type)
    return inner_cs
  return 'void'


def _get_struct_typename(struct_decl):
  name = _strip_struct_prefix(struct_decl.declname)
  if name in _STRUCT_NAME_OVERRIDES:
    return _STRUCT_NAME_OVERRIDES[name]
  return name


def _is_struct_type(type_name):
  return type_name.endswith('_') and type_name not in (
      set(C_TO_CS_TYPE.values()) | _BLITTABLE_TYPES
  )


def _is_omitted_struct(struct_name):
  for omitted in _OPAQUE_STRUCTS:
    if struct_name.startswith(omitted):
      return True
  return False


def _is_omitted_function(func_name):
  for omitted in _OMITTED_FUNCTIONS:
    if func_name.startswith(omitted):
      return True
  return False


def _parse_constants_from_headers(header_files):
  """Parses #define constants from MuJoCo header files."""
  constants = {}
  define_with_value = re.compile(
      r'^[ \t]*#[ \t]*define[ \t]+(\w+)[ \t]+(\S.+?)(?:[ \t]*//.*)?$',
      re.MULTILINE,
  )
  define_no_value = re.compile(
      r'^[ \t]*#[ \t]*define[ \t]+(\w+)[ \t]*$', re.MULTILINE
  )

  for header_file in header_files:
    try:
      with open(header_file, 'r') as f:
        content = f.read()
    except (IOError, OSError):
      continue

    for match in define_no_value.finditer(content):
      name = match.group(1)
      if name.startswith('_'):
        continue
      constants[name] = True

    for match in define_with_value.finditer(content):
      name = match.group(1)
      value_str = match.group(2).strip()

      if name.startswith('_'):
        continue
      if '(' in name:
        continue

      if value_str.startswith('(') and value_str.endswith(')'):
        value_str = value_str[1:-1].strip()

      clean_val = value_str.rstrip('UuFf')

      try:
        value = float(clean_val)
        if (
            value == int(value)
            and '.' not in clean_val
            and 'e' not in clean_val.lower()
        ):
          constants[name] = int(value)
        else:
          constants[name] = value
        continue
      except ValueError:
        pass

      try:
        constants[name] = int(clean_val, 0)
        continue
      except ValueError:
        pass

      try:
        evaled = eval(clean_val, {'__builtins__': {}}, constants)  # pylint: disable=eval-used
        if isinstance(evaled, (int, float)):
          constants[name] = evaled
      except Exception:  # pylint: disable=broad-except
        pass

  return constants


_COPYRIGHT_HEADER = """// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ==============================================================================
// THIS FILE IS AUTO-GENERATED
"""


def _write_header(output):
  """Writes the file header including copyright and imports."""
  output.write(_COPYRIGHT_HEADER)
  output.write(
      '\n'.join(
          _BOILERPLATE_IMPORTS
          + ['namespace Mujoco {', 'public static class MujocoLib {\n']
      )
  )


def _write_footer(output):
  output.write('}\n}\n')


def _write_constants(output, constants):
  output.write('\n// ' + '-' * 34 + 'Constants' + '-' * 34 + '\n\n')
  for name, value in constants.items():
    const_type = type(value).__name__
    const_type = PYTHON_TYPE_TO_CS_TYPE.get(const_type, const_type)
    const_value = PYTHON_VALUE_TO_CS_VALUE.get(value, value)
    if name == 'mjVERSION_HEADER':
      output.write('// copy' + 'bara:strip_begin\n')
      output.write(f'public const {const_type} {name} = {const_value};\n')
      output.write('/* copy' + 'bara:strip_end_and_replace\n')
      output.write(
          f'public const {const_type} {name} = __COPYBARA'
          + '_MUJOCO_VERSION_NUMERIC__;\n'
      )
      output.write('*/\n')
    else:
      output.write(f'public const {const_type} {name} = {const_value};\n')
  output.write('\n')


def _write_enums(output):
  output.write('\n// ' + '-' * 36 + 'Enums' + '-' * 36 + '\n')
  for enum_name, enum_decl in introspect_enums.ENUMS.items():
    output.write(f'public enum {enum_name} : int{{\n')
    for member_name, member_value in enum_decl.values.items():
      output.write(f'  {member_name} = {member_value},\n')
    output.write('}\n')


def _field_to_cs_member(field, known_structs):
  if isinstance(field, ast_nodes.AnonymousUnionDecl):
    return _anonymous_union_to_cs(field, known_structs)
  if isinstance(field, ast_nodes.AnonymousStructDecl):
    return _anonymous_struct_to_cs(field, known_structs)

  name = _mangle_name(field.name)
  field_type = field.type

  if isinstance(
      field_type, (ast_nodes.AnonymousStructDecl, ast_nodes.AnonymousUnionDecl)
  ):
    return f'  public {field.name} {name};\n'

  if isinstance(field_type, ast_nodes.ValueType):
    cs_type = _resolve_type_name(field_type.name)
    return f'  public {cs_type} {name};\n'

  elif isinstance(field_type, ast_nodes.ArrayType):
    inner_type = field_type.inner_type
    if isinstance(inner_type, ast_nodes.ValueType):
      cs_type = _resolve_type_name(inner_type.name)
    elif isinstance(inner_type, ast_nodes.PointerType):
      cs_type = _resolve_cs_type(inner_type)
    else:
      cs_type = 'int'

    extents = field_type.extents
    total_size = ' * '.join(str(e) for e in extents)

    if cs_type in _ALLOWED_FIXED_ARRAYS:
      return f'  public fixed {cs_type} {name}[{total_size}];\n'
    elif cs_type in known_structs or cs_type in _BLITTABLE_TYPES:
      result = ''
      cs_display = cs_type if cs_type != 'void' else 'IntPtr'
      if len(extents) == 1:
        for i in range(extents[0]):
          result += f'  public {cs_display} {name}{i};\n'
      elif len(extents) == 2:
        for i in range(extents[0]):
          for j in range(extents[1]):
            result += f'  public {cs_display} {name}{i}_{j};\n'
      return result
    else:
      return f'  public fixed {cs_type} {name}[{total_size}];\n'

  elif isinstance(field_type, ast_nodes.PointerType):
    inner = field_type.inner_type
    if isinstance(inner, ast_nodes.ValueType):
      if _is_opaque_type(inner.name):
        return f'  public void* {name};\n'
      cs_type = _resolve_type_name(inner.name)
      if inner.name == 'char' and not field_type.is_const:
        return f'  public char* {name};\n'
      return f'  public {cs_type}* {name};\n'
    elif isinstance(inner, ast_nodes.PointerType):
      inner_cs = _resolve_cs_type(inner)
      return f'  public {inner_cs}* {name};\n'
    else:
      return f'  public void* {name};\n'

  return f'  public int {name};\n'


def _anonymous_union_to_cs(union_decl, known_structs):
  result = ''
  for sub_field in union_decl.fields:
    result += _field_to_cs_member(sub_field, known_structs)
  return result


def _anonymous_struct_to_cs(struct_decl, known_structs):
  result = ''
  for sub_field in struct_decl.fields:
    result += _field_to_cs_member(sub_field, known_structs)
  return result


def _emit_anonymous_sub_structs(output, struct_decl, known_structs):
  """Emits separate struct declarations for anonymous sub-structs."""
  for field in struct_decl.fields:
    if not isinstance(field, ast_nodes.StructFieldDecl):
      continue
    if not isinstance(
        field.type,
        (ast_nodes.AnonymousStructDecl, ast_nodes.AnonymousUnionDecl),
    ):
      continue
    sub_type_name = field.name
    output.write('\n[StructLayout(LayoutKind.Sequential)]\n')
    output.write(f'public unsafe struct {sub_type_name} {{\n')
    for sub_field in field.type.fields:
      if isinstance(sub_field, ast_nodes.StructFieldDecl):
        output.write(_field_to_cs_member(sub_field, known_structs))
      elif isinstance(
          sub_field,
          (ast_nodes.AnonymousUnionDecl, ast_nodes.AnonymousStructDecl),
      ):
        for inner_field in sub_field.fields:
          output.write(_field_to_cs_member(inner_field, known_structs))
    output.write('}\n')
    known_structs.add(sub_type_name)


def _write_structs(output, known_structs):
  """Writes struct declarations to a file."""
  output.write('\n// ' + '-' * 31 + 'struct declarations' + '-' * 27 + '\n')
  for struct_name, struct_decl in introspect_structs.STRUCTS.items():
    if _is_omitted_struct(struct_name):
      continue

    typename = _get_struct_typename(struct_decl)

    _emit_anonymous_sub_structs(output, struct_decl, known_structs)

    output.write('\n[StructLayout(LayoutKind.Sequential)]\n')
    output.write(f'public unsafe struct {typename} {{\n')

    for field in struct_decl.fields:
      if isinstance(field, ast_nodes.StructFieldDecl):
        if field.array_extent is not None:
          cs_type = _resolve_cs_type(field.type)
          name = _mangle_name(field.name)
          output.write(f'  public {cs_type} {name};\n')
        else:
          output.write(_field_to_cs_member(field, known_structs))
      elif isinstance(
          field, (ast_nodes.AnonymousUnionDecl, ast_nodes.AnonymousStructDecl)
      ):
        for sub_field in field.fields:
          output.write(_field_to_cs_member(sub_field, known_structs))

    output.write('}\n')
    known_structs.add(typename)

  for stub in _STUB_STRUCTS:
    if stub not in known_structs:
      output.write(f'\n[StructLayout(LayoutKind.Sequential)]\n')
      output.write(f'public unsafe struct {stub} {{}}\n')
      known_structs.add(stub)


def _func_param_to_cs(param):
  param_type = param.type
  param_name = _mangle_name(param.name)

  if isinstance(param_type, ast_nodes.ValueType):
    cs_type = _resolve_type_name(param_type.name)
    return cs_type, param_name, None

  elif isinstance(param_type, ast_nodes.PointerType):
    inner = param_type.inner_type
    if isinstance(inner, ast_nodes.ValueType):
      if inner.name == 'char':
        if param_type.is_const or (
            hasattr(inner, 'is_const') and inner.is_const
        ):
          return 'string', param_name, 'MarshalAs(UnmanagedType.LPStr)'
        else:
          return 'StringBuilder', param_name, None
      if _is_opaque_type(inner.name):
        return 'void*', param_name, None
      cs_type = _resolve_type_name(inner.name)
      return cs_type + '*', param_name, None
    elif isinstance(inner, ast_nodes.PointerType):
      inner_cs = _resolve_cs_type(inner)
      return inner_cs + '*', param_name, None
    elif isinstance(inner, ast_nodes.ArrayType):
      inner_cs = _resolve_cs_type(inner.inner_type)
      return inner_cs + '*', param_name, None
    else:
      return 'void*', param_name, None

  elif isinstance(param_type, ast_nodes.ArrayType):
    inner_cs = _resolve_cs_type(param_type.inner_type)
    return inner_cs + '*', param_name, None

  return 'int', param_name, None


def _func_return_to_cs(return_type):
  if isinstance(return_type, ast_nodes.ValueType):
    cs_type = _resolve_type_name(return_type.name)
    return cs_type, None

  elif isinstance(return_type, ast_nodes.PointerType):
    inner = return_type.inner_type
    if isinstance(inner, ast_nodes.ValueType):
      if inner.name == 'char':
        return 'IntPtr', None
      if _is_opaque_type(inner.name):
        return 'void*', None
      cs_type = _resolve_type_name(inner.name)
      return cs_type + '*', None
    elif isinstance(inner, ast_nodes.PointerType):
      inner_cs = _resolve_cs_type(inner)
      return inner_cs + '*', None
    return 'void*', None

  return 'void', None


def _write_functions(output):
  output.write('\n// ' + '-' * 30 + 'Function declarations' + '-' * 26 + '\n')
  for func_name, func_decl in introspect_functions.FUNCTIONS.items():
    if _is_omitted_function(func_name):
      continue

    name = _mangle_name(func_name)

    args = []
    for param in func_decl.parameters:
      cs_type, param_name, annotation = _func_param_to_cs(param)
      if annotation:
        args.append(f'[{annotation}]{cs_type} {param_name}')
      else:
        args.append(f'{cs_type} {param_name}')
    args_str = ', '.join(args)

    return_type, return_annotation = _func_return_to_cs(func_decl.return_type)

    output.write(
        '\n[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]\n'
    )
    if return_annotation:
      output.write(f'[{return_annotation}]\n')
    output.write(
        f'public static unsafe extern {return_type} {name}({args_str});\n'
    )


def main(argv):
  del argv

  constants = {}
  if _HEADER_FILES.value:
    header_files = _HEADER_FILES.value.split()
    constants = _parse_constants_from_headers(header_files)

  known_structs = set()
  for struct_name, struct_decl in introspect_structs.STRUCTS.items():
    if not _is_omitted_struct(struct_name):
      known_structs.add(_get_struct_typename(struct_decl))

  output = sys.stdout

  _write_header(output)
  if constants:
    _write_constants(output, constants)
  _write_enums(output)
  _write_structs(output, known_structs)
  _write_functions(output)
  _write_footer(output)


if __name__ == '__main__':
  app.run(main)
