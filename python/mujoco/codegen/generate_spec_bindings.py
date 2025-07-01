# Copyright 2024 DeepMind Technologies Limited
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
"""Generates the bindings for the MuJoCo specs."""

from collections.abc import Sequence

from absl import app

from introspect import ast_nodes
from introspect import structs


SCALAR_TYPES = {'int', 'double', 'float', 'mjtByte', 'mjtNum'}

# pylint: disable=bad-whitespace
# key, parent, default, listname, objtype
SPECS = [
    ('mjsBody',     'Body', True,  'bodies',     'mjOBJ_BODY'),
    ('mjsSite',     'Body', True,  'sites',      'mjOBJ_SITE'),
    ('mjsGeom',     'Body', True,  'geoms',      'mjOBJ_GEOM'),
    ('mjsJoint',    'Body', True,  'joints',     'mjOBJ_JOINT'),
    ('mjsCamera',   'Body', True,  'cameras',    'mjOBJ_CAMERA'),
    ('mjsFrame',    'Body', True,  'frames',     'mjOBJ_FRAME'),
    ('mjsLight',    'Body', True,  'lights',     'mjOBJ_LIGHT'),
    ('mjsFlex',     'Spec', False, 'flexes',     'mjOBJ_FLEX'),
    ('mjsMesh',     'Spec', True,  'meshes',     'mjOBJ_MESH'),
    ('mjsSkin',     'Spec', False, 'skins',      'mjOBJ_SKIN'),
    ('mjsHField',   'Spec', False, 'hfields',    'mjOBJ_HFIELD'),
    ('mjsTexture',  'Spec', False, 'textures',   'mjOBJ_TEXTURE'),
    ('mjsMaterial', 'Spec', True,  'materials',  'mjOBJ_MATERIAL'),
    ('mjsPair',     'Spec', True,  'pairs',      'mjOBJ_PAIR'),
    ('mjsEquality', 'Spec', True,  'equalities', 'mjOBJ_EQUALITY'),
    ('mjsTendon',   'Spec', True,  'tendons',    'mjOBJ_TENDON'),
    ('mjsActuator', 'Spec', True,  'actuators',  'mjOBJ_ACTUATOR'),
    ('mjsSensor',   'Spec', False, 'sensors',    'mjOBJ_SENSOR'),
    ('mjsNumeric',  'Spec', False, 'numerics',   'mjOBJ_NUMERIC'),
    ('mjsText',     'Spec', False, 'texts',      'mjOBJ_TEXT'),
    ('mjsTuple',    'Spec', False, 'tuples',     'mjOBJ_TUPLE'),
    ('mjsKey',      'Spec', False, 'keys',       'mjOBJ_KEY'),
    ('mjsExclude',  'Spec', False, 'excludes',   'mjOBJ_EXCLUDE'),
    ('mjsPlugin',   'Spec', False, 'plugins',    'mjOBJ_PLUGIN'),
]
# pylint: enable=bad-whitespace


def _value_binding_code(
    field: ast_nodes.ValueType, classname: str = '', varname: str = ''
) -> str:
  """Creates a string that defines Python bindings for a value type."""
  fulltype = field.name
  if field.name not in SCALAR_TYPES:
    fulltype += '&'
  fullvarname = varname
  rawclassname = classname.replace('mjs', 'raw::Mjs')
  if classname == 'mjSpec':  # raw mjSpec has a wrapper
    rawclassname = classname.replace('mjS', 'MjS')
    fullvarname = 'ptr->' + varname
  if field.name.startswith('mjs'):  # all other mjs are raw structs
    fulltype = field.name.replace('mjs', 'raw::Mjs')
    if (
        field.name == 'mjsPlugin'
        or field.name == 'mjsOrientation'
        or field.name == 'mjsCompiler'
    ):
      fulltype = fulltype + '&'  # plugin, orientation, compiler aren't pointers
    else:
      fulltype = fulltype + '*'
  # non-mjs structs
  rawclassname = rawclassname.replace('mjOption', 'raw::MjOption')
  rawclassname = rawclassname.replace('mjVisual', 'raw::MjVisual')
  rawclassname = rawclassname.replace('mjStatistic', 'raw::MjStatistic')
  fulltype = fulltype.replace('mjOption', 'raw::MjOption')
  fulltype = fulltype.replace('mjVisual', 'raw::MjVisual')
  fulltype = fulltype.replace('mjStatistic', 'raw::MjStatistic')
  element = '.element' if fullvarname == 'plugin' else ''

  def_property_args = (
      f'"{varname}"',
      f"""[]({rawclassname}& self) -> {fulltype} {{
        return self.{fullvarname};
      }}""",
      f"""[]({rawclassname}& self, {fulltype} {varname}) {{
        self.{fullvarname}{element} = {varname}{element};
      }}""",
  )

  if field.name not in SCALAR_TYPES:
    def_property_args += ('py::return_value_policy::reference_internal',)

  return f'{classname}.def_property({",".join(def_property_args)});'


def _struct_binding_code(
    field: ast_nodes.AnonymousStructDecl, classname: str = '', varname: str = ''
) -> str:
  """Creates a string that declares Python bindings for an anonymous struct."""
  code = ''
  name = classname + varname.title()
  # explicitly generate for nested fields with arrays
  if any(
      isinstance(f, ast_nodes.StructFieldDecl)
      and isinstance(f.type, ast_nodes.ArrayType)
      for f in field.fields
  ):
    for subfield in field.fields:
      code += _binding_code(subfield, name)
  # generate for the struct itself
  field = ast_nodes.ValueType(name=name)
  code += _value_binding_code(field, classname, varname)
  return code


def _array_binding_code(
    field: ast_nodes.ArrayType, classname: str = '', varname: str = ''
) -> str:
  """Creates a string that declares Python bindings for an array type."""
  if len(field.extents) > 1:
    raise NotImplementedError()
  innertype = field.inner_type.decl()
  rawclassname = classname.replace('mjs', 'raw::Mjs')
  rawclassname = rawclassname.replace('mjOption', 'raw::MjOption')
  rawclassname = rawclassname.replace('mjVisual', 'raw::MjVisual')
  rawclassname = rawclassname.replace('mjStatistic', 'raw::MjStatistic')
  fullvarname = varname
  if classname == 'mjSpec':  # raw mjSpec has a wrapper
    rawclassname = classname.replace('mjS', 'MjS')
    fullvarname = 'ptr->' + varname
  if innertype == 'double' or innertype == 'mjtNum':
    innertype = 'MjDouble'  # custom Eigen type
  elif innertype == 'float':
    innertype = 'MjFloat'  # custom Eigen type
  elif innertype == 'int':
    innertype = 'MjInt'  # custom Eigen type
  elif innertype == 'char':
    # char array special case
    return f"""\
  {classname}.def_property(
    "{varname}",
    []({rawclassname}& self) -> MjTypeVec<char> {{
      return MjTypeVec<char>(self.{fullvarname}, {field.extents[0]});
    }},
    []({rawclassname}& self, py::object rhs) {{
      int i = 0;
      for (auto val : rhs) {{
        self.{fullvarname}[i++] = py::cast<char>(val);
      }}
    }}, py::return_value_policy::move);"""
  # all other array types
  return f"""\
  {classname}.def_property(
      "{varname}",
      []({rawclassname}& self) -> {innertype}{field.extents[0]} {{
        return {innertype}{field.extents[0]}(self.{fullvarname});
    }},
      []({rawclassname}& self, {innertype}Ref{field.extents[0]} {varname}) {{
        {innertype}{field.extents[0]}(self.{fullvarname}) = {varname};
    }}, py::return_value_policy::reference_internal);"""


def _ptr_binding_code(
    field: ast_nodes.PointerType, classname: str = '', varname: str = ''
) -> str:
  """Creates a string that declares Python bindings for a pointer type."""
  vartype = field.inner_type.decl()
  rawclassname = classname.replace('mjs', 'raw::Mjs')
  fullvarname = varname
  if classname == 'mjSpec':  # raw mjSpec has a wrapper
    rawclassname = classname.replace('mjS', 'MjS')
    fullvarname = 'ptr->' + varname
  if vartype == 'mjsElement':  # this is ignored by the caller
    return 'mjsElement'
  if vartype.startswith('mjs'):  # for structs, use the value case
    return _value_binding_code(field.inner_type, classname, varname)
  elif vartype == 'mjString':  # C++ string -> Python string
    return f"""\
  {classname}.def_property(
      "{varname}",
      []({rawclassname}& self) -> std::string_view {{
        return *self.{fullvarname};
      }},
      []({rawclassname}& self, std::string_view {varname}) {{
        *(self.{fullvarname}) = {varname};
    }});"""
  elif (  # C++ vectors of values -> Python array
      vartype == 'mjDoubleVec'
      or vartype == 'mjFloatVec'
      or vartype == 'mjIntVec'
  ):
    vartype = vartype.replace('mj', '').replace('Vec', '').lower()
    return f"""\
  {classname}.def_property(
    "{varname}",
    []({rawclassname}& self) -> py::array_t<{vartype}> {{
        return py::array_t<{vartype}>(self.{fullvarname}->size(),
                                      self.{fullvarname}->data());
      }},
    []({rawclassname}& self, py::object rhs) {{
        self.{fullvarname}->clear();
        self.{fullvarname}->reserve(py::len(rhs));
        for (auto val : rhs) {{
          self.{fullvarname}->push_back(py::cast<{vartype}>(val));
      }}
    }}, py::return_value_policy::reference_internal);"""
  elif vartype == 'mjByteVec':
    return f"""\
  {classname}.def_property(
    "{varname}",
    []({rawclassname}& self) -> MjTypeVec<std::byte> {{
        return MjTypeVec<std::byte>(self.{fullvarname}->data(),
                                    self.{fullvarname}->size());
      }},
    []({rawclassname}& self, py::bytes& rhs) {{
        self.{fullvarname}->clear();
        self.{fullvarname}->reserve(py::len(rhs));
        std::string_view rhs_view = py::cast<std::string_view>(rhs);
        for (auto val : rhs_view) {{
          self.{fullvarname}->push_back(static_cast<std::byte>(val));
        }}
    }}, py::return_value_policy::move);"""
  elif vartype == 'mjStringVec':
    return f"""\
  {classname}.def_property(
    "{varname}",
    []({rawclassname}& self) -> MjTypeVec<std::string> {{
        return MjTypeVec<std::string>(self.{fullvarname}->data(),
                                      self.{fullvarname}->size());
      }},
    []({rawclassname}& self, py::object rhs) {{
        self.{fullvarname}->clear();
        self.{fullvarname}->reserve(py::len(rhs));
        for (auto val : rhs) {{
          self.{fullvarname}->push_back(py::cast<std::string>(val));
      }}
    }}, py::return_value_policy::move);"""
  elif 'VecVec' in vartype:  # C++ vector of vectors -> Python list of lists
    vartype = vartype.replace('mj', '').replace('VecVec', '').lower()
    return f"""\
  {classname}.def_property(
    "{varname}",
    []({rawclassname}& self) -> py::list {{
        py::list list;
        for (auto inner_vec : *self.{fullvarname}) {{
          py::list inner_list;
          for (auto val : inner_vec) {{
            inner_list.append(val);
          }}
          list.append(inner_list);
        }}
        return list;
      }},
    []({rawclassname}& self, py::object rhs) {{
        self.{fullvarname}->clear();
        self.{fullvarname}->reserve(py::len(rhs));
        for (auto inner_list : rhs) {{
          auto inner_vec = py::cast<std::vector<{vartype}>>(inner_list);
          self.{fullvarname}->push_back(inner_vec);
        }}
    }}, py::return_value_policy::reference_internal);"""

  raise NotImplementedError(
      'Unsupported array type: ' + vartype + ' in ' + classname
  )


def _binding_code(field: ast_nodes.StructFieldDecl, key: str) -> str:
  if isinstance(field.type, ast_nodes.ValueType):
    return _value_binding_code(field.type, key, field.name)
  elif isinstance(field.type, ast_nodes.AnonymousStructDecl):
    return _struct_binding_code(field.type, key, field.name)
  elif isinstance(field.type, ast_nodes.PointerType):
    return _ptr_binding_code(field.type, key, field.name)
  elif isinstance(field.type, ast_nodes.ArrayType):
    return _array_binding_code(field.type, key, field.name)
  return ''


def generate() -> None:
  for key in structs.STRUCTS.keys():
    if (
        key.startswith('mjs')
        or key in ['mjSpec', 'mjOption', 'mjVisual', 'mjStatistic']
    ) and key != 'mjsElement':
      print('\n  // ' + key)
      for field in structs.STRUCTS[key].fields:
        code = _binding_code(field, key)
        if code != 'mjsElement':
          print(code)


def generate_add() -> None:
  """Generate add constructors with optional keyword arguments."""
  for key, parent, default, listname, objtype in SPECS:

    def _field(f: ast_nodes.StructFieldDecl):
      if f.type == ast_nodes.PointerType(
          inner_type=ast_nodes.ValueType(name='mjsElement')
      ):
        return '', '', ''
      elif f.type == ast_nodes.ValueType(name='mjsPlugin'):
        return f'set_plugin(out->{f.name});', 'plugin', f.name
      elif f.type == ast_nodes.ValueType(name='mjsOrientation'):
        return (
            (
                f'set_orientation(out->{f.name},'
                f' "{"iaxisangle" if f.name == "ialt" else "axisangle"}",'
                f' "{"ixyaxes" if f.name == "ialt" else "xyaxes"}",'
                f' "{"izaxis" if f.name == "ialt" else "zaxis"}",'
                f' "{"ieuler" if f.name == "ialt" else "euler"}");'
            ),
            'orientation',
            ['iaxisangle', 'ixyaxes', 'izaxis', 'ieuler']
            if f.name == 'ialt'
            else ['axisangle', 'xyaxes', 'zaxis', 'euler'],
        )
      elif f.type == ast_nodes.PointerType(
          inner_type=ast_nodes.ValueType(name='mjString')
      ):
        return f'set_string("{f.name}", out->{f.name});', 'string', f.name
      elif isinstance(f.type, ast_nodes.PointerType):
        return f'set_vec("{f.name}", out->{f.name});', 'vec', f.name
      elif isinstance(f.type, ast_nodes.ArrayType):
        return (
            f'set_array("{f.name}", out->{f.name}, {f.type.extents[0]});',
            'array',
            f.name,
        )
      elif isinstance(f.type, ast_nodes.ValueType):
        return f'set_value("{f.name}", out->{f.name});', 'value', f.name
      else:
        return '', '', ''

    code_field = ''
    set_types = []
    names = []
    for field in structs.STRUCTS[key].fields:
      line, set_type, name = _field(field)
      if line:
        code_field = code_field + '\n        ' + line
        set_types.append(set_type)
        if set_type == 'orientation':
          names.extend(name)
        else:
          names.append(name)

    # assemble
    elem = key.removeprefix('mjs')
    elemlower = elem.lower()
    titlecase = 'Mjs' + elem

    # function definition and call to mjs_add_
    if parent == 'Spec':
      if default:
        code = f"""
          {'mj' + parent}.def("add_{elemlower}", []({'Mj' + parent}& self,
            raw::MjsDefault* default_, py::kwargs kwargs) -> raw::{titlecase}* {{
            auto out = mjs_add{elem}(self.ptr, default_);
        """
      else:
        code = f"""
          {'mj' + parent}.def("add_{elemlower}", []({'Mj' + parent}& self, py::kwargs kwargs) -> raw::{titlecase}* {{
            auto out = mjs_add{elem}(self.ptr);
        """
    elif parent == 'Body':
      if key == 'mjsFrame':
        code = f"""
          {'mjs' + parent}.def("add_{elemlower}", []({'raw::Mjs' + parent}& self,
            raw::MjsFrame* parentframe_, py::kwargs kwargs) -> raw::{titlecase}* {{
            auto out = mjs_add{elem}(&self, parentframe_);
        """
      else:
        code = f"""
          {'mjs' + parent}.def("add_{elemlower}", []({'raw::Mjs' + parent}& self,
            raw::MjsDefault* default_, py::kwargs kwargs) -> raw::{titlecase}* {{
            auto out = mjs_add{elem}(&self, default_);
        """
    else:
      raise NotImplementedError(f'{parent} parent is not implement.')

    # check for valid kwargs
    code += '\n        std::set<std::string> valid_kwargs = {'
    valid_kwargs = ''
    for i, name in enumerate(names):
      valid_kwargs += f'"{name}"'
      if i != len(names) - 1:
        valid_kwargs += ', '
    code += valid_kwargs + '};'

    code += f"""\n
        py::dict kwarg_dict = kwargs;
        for (auto item: kwarg_dict) {{
          std::string key = py::str(item.first);
          if (valid_kwargs.count(key) == 0) {{
            throw pybind11::type_error("Invalid "
              + key
              + " keyword argument. Valid options are: {", ".join(names)}.");
          }}
        }}
    """

    # include helper functions
    if set_types:
      for t in set(set_types):
        if t == 'orientation':
          code += """\n
          auto set_orientation = [&kwargs](raw::MjsOrientation& orientation,
            const char* axisangle,
            const char* xyaxes,
            const char* zaxis,
            const char* euler) {
            int nrepresentation = 0;
            bool has_axisangle = kwargs.contains(axisangle);
            nrepresentation += has_axisangle;
            bool has_xyaxes = kwargs.contains(xyaxes);
            nrepresentation += has_xyaxes;
            bool has_zaxis = kwargs.contains(zaxis);
            nrepresentation += has_zaxis;
            bool has_euler = kwargs.contains(euler);
            nrepresentation += has_euler;

            if (nrepresentation == 0) {
              return;
            } else if (nrepresentation > 1) {
              throw pybind11::value_error("Only one of: "
                + std::string(axisangle) + ", "
                + std::string(xyaxes) + ", "
                + std::string(zaxis)
                + ", or"
                + std::string(euler)
                + " can be set.");
            }

            auto set_array = [&kwargs](const char* str, double* des, int size) {
              try {
                std::vector<double> array = kwargs[str].cast<std::vector<double>>();
                if (array.size() != size) {
                  throw pybind11::value_error(std::string(str)
                    + " should be a list/array of size "
                    + std::to_string(size)
                    + ".");
                }
                int idx = 0;
                for (auto val : array) {
                  des[idx++] = val;
                }
              } catch (const py::cast_error &e) {
                throw pybind11::value_error(std::string(str)
                  + " should be a list/array.");
              }
            };

            if (has_axisangle) {
              set_array(axisangle, orientation.axisangle, 4);
              orientation.type = mjORIENTATION_AXISANGLE;
            } else if (has_xyaxes) {
              set_array(xyaxes, orientation.xyaxes, 6);
              orientation.type = mjORIENTATION_XYAXES;
            } else if (has_zaxis) {
              set_array(zaxis, orientation.zaxis, 3);
              orientation.type = mjORIENTATION_ZAXIS;
            } else if (has_euler) {
              set_array(euler, orientation.euler, 3);
              orientation.type = mjORIENTATION_EULER;
            }
          };
          """
        elif t == 'plugin':
          code += """\n
          auto set_plugin = [&kwargs](raw::MjsPlugin& plugin) {
            if (kwargs.contains("plugin")) {
              std::optional<raw::MjsPlugin> input = kwargs["plugin"].cast<raw::MjsPlugin>();
              if (input.has_value()) {
                try {
                  plugin.name = input->name;
                } catch (const py::cast_error &e) {
                  throw pybind11::value_error("plugin.name should be a string.");
                }
                try {
                  plugin.plugin_name = input->plugin_name;
                } catch (const py::cast_error &e) {
                  throw pybind11::value_error("plugin.instance_name should be a string.");
                }
                try {
                  plugin.active = input->active;
                } catch (const py::cast_error &e) {
                  throw pybind11::value_error("plugin.active should be an mjtByte.");
                }
                try {
                  plugin.info = input->info;
                } catch (const py::cast_error &e) {
                  throw pybind11::value_error("plugin.info should be a string.");
                }
              }
            }
          };
          """
        elif t == 'string':
          code += """\n
          auto set_string = [&kwargs](const char* str, std::basic_string<char>* des) {
            if (kwargs.contains(str)) {
              try {
                *des = kwargs[str].cast<std::string>();
              } catch (const py::cast_error &e) {
                throw pybind11::value_error(std::string(str) + " should be a string.");
              }
            }
          };
          """
        elif t == 'vec':
          code += """\n
          auto set_vec = [&kwargs](const char* str, auto&& des) {
            if (kwargs.contains(str)) {
              try {
                using T = typename std::decay_t<decltype(*des)>::value_type;
                std::vector<T> vec = kwargs[str].cast<std::vector<T>>();
                des->clear();
                des->reserve(vec.size());
                for (auto val : vec) {
                  des->push_back(val);
                }
              } catch (const py::cast_error &e) {
                throw pybind11::value_error(std::string(str) + " has the wrong type.");
              }
            }
          };
          """
        elif t == 'array':
          code += """\n
          auto set_array = [&kwargs](const char* str, auto&& des, int size) {
            if (kwargs.contains(str)) {
              try {
                using T = std::remove_pointer_t<std::decay_t<decltype(des)>>;
                std::vector<T> array = kwargs[str].cast<std::vector<T>>();
                if (array.size() != size) {
                  throw pybind11::value_error(std::string(str)
                    + " should be a list/array of size "
                    + std::to_string(size)
                    + ".");
                }
                int idx = 0;
                for (auto val : array) {
                  des[idx++] = val;
                }
              } catch (const py::cast_error &e) {
                throw pybind11::value_error(std::string(str) + " should be a list/array.");
              }
            }
          };
          """
        elif t == 'value':
          code += """\n
          auto set_value = [&kwargs](const char* str, auto&& des) {
            if (kwargs.contains(str)) {
              try {
                using T = std::decay_t<decltype(des)>;
                des = kwargs[str].cast<T>();
              } catch (const py::cast_error &e) {
                throw pybind11::value_error(std::string(str) + " is the wrong type.");
              }
            }
          };
          """

    code += code_field
    code += f"""\n
        return out;
      }},
      {'py::arg_v("default", nullptr),' if default else ''}
      py::return_value_policy::reference_internal);
    """

    code += f"""\n
      mjSpec.def_property_readonly(
        "{listname}",
        [](MjSpec& self) -> py::list {{
          py::list list;
          raw::MjsElement* el = mjs_firstElement(self.ptr, {objtype});
          while (el) {{
            list.append(mjs_as{key[3:]}(el));
            el = mjs_nextElement(self.ptr, el);
          }}
          return list;
        }},
        py::return_value_policy::reference_internal);
    """

    print(code)


def generate_find() -> None:
  """Generate find functions."""
  for key, _, _, _, objtype in SPECS:
    elem = key.removeprefix('mjs')
    elemlower = elem.lower()
    titlecase = 'Mjs' + elem
    code = f"""\n
      mjSpec.def("{elemlower}",
      [](MjSpec& self, std::string& name) -> raw::{titlecase}* {{
        return mjs_as{elem}(
            mjs_findElement(self.ptr, {objtype}, name.c_str()));
      }}, py::return_value_policy::reference_internal);
    """
    print(code)


def generate_signature() -> None:
  """Generate signature functions."""
  for key, _, _, _, _ in SPECS:
    elem = key.removeprefix('mjs')
    titlecase = 'Mjs' + elem
    code = f"""\n
      {key}.def_property_readonly("signature",
      [](raw::{titlecase}& self) -> uint64_t {{
        return mjs_getSpec(self.element)->element->signature;
      }});
    """
    print(code)


def generate_id() -> None:
  """Generate id functions."""
  for key, _, _, _, _ in SPECS:
    if key == 'mjsPlugin':
      continue
    elem = key.removeprefix('mjs')
    titlecase = 'Mjs' + elem
    code = f"""\n
      {key}.def_property_readonly("id",
      [](raw::{titlecase}& self) -> int {{
        return mjs_getId(self.element);
      }});
    """
    print(code)


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')
  generate()
  generate_add()
  generate_find()
  generate_signature()
  generate_id()


if __name__ == '__main__':
  app.run(main)
