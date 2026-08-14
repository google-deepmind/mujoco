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
"""Tests for the MJCF schema definition language parser."""

import os
import sys
import unittest as googletest
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
sys.path.insert(0, os.path.join(_REPO_ROOT, 'doc', 'generate'))
import mjcf_schema

GOOD = '''
enum geomtype : mjtGeom {          # geom shapes
  plane   = mjGEOM_PLANE
  sphere  = mjGEOM_SPHERE
  "2d"    = mjGEOM_PLANE
}

enum onoff {
  false = 0
  true  = 1
}

group orientation variant {        # at most one spelling
  quat      : double[4] = {1, 0, 0, 0}
  axisangle : double[4]            # (x, y, z, angle)
  euler     : double[3]
}

group posed {
  pos : double[3] = {0, 0, 0}
  use orientation
}

element defaults {
  class    : id<defaults>                      # name of this class
}

element geom : mjsGeom {           # geometric entity
  use posed
  name     : id<geom>                          # element name
  class    : ref<defaults> (field=classname)   # defaults class
  type     : enum<geomtype> = sphere           # geom shape
  condim   : int = 3
  size     : double[0..3]                      # type-specific size
  friction : double[1..3] = {1, 0.005, 0.0001} # slide, roll, spin
  solref   : double[0..mjNREF]
  eulerseq : string = "xyz" (pattern="[xyzXYZ]{3}")
  margin   : double = 0 (nodefault)
  file     : string (required)
  user     : double[]                          # user data
  child geom *                                 # nested geoms
  child defaults R
}
'''


class ParserTest(googletest.TestCase):

  def parse(self, text):
    return mjcf_schema.parse_string(text)

  def error(self, text):
    with self.assertRaises(mjcf_schema.SchemaError) as ctx:
      self.parse(text)
    return str(ctx.exception)

  def test_good_schema_parses(self):
    schema = self.parse(GOOD)
    self.assertEqual(set(schema.enums), {'geomtype', 'onoff'})
    self.assertEqual(set(schema.groups), {'orientation', 'posed'})
    self.assertEqual(set(schema.elements), {'geom', 'defaults'})

  def test_enum(self):
    schema = self.parse(GOOD)
    enum = schema.enums['geomtype']
    self.assertEqual(enum.ctype, 'mjtGeom')
    self.assertEqual(enum.items[0], ('plane', 'mjGEOM_PLANE'))
    self.assertEqual(enum.items[2], ('2d', 'mjGEOM_PLANE'))
    self.assertEqual(enum.doc, 'geom shapes')
    self.assertIsNone(schema.enums['onoff'].ctype)
    self.assertEqual(schema.enums['onoff'].items[0], ('false', '0'))

  def test_groups_and_expansion(self):
    schema = self.parse(GOOD)
    self.assertTrue(schema.groups['orientation'].variant)
    self.assertFalse(schema.groups['posed'].variant)
    names = [a.name for a in schema.expanded_attrs(schema.elements['geom'])]
    # posed expands to pos + orientation members, in order, before own attrs.
    self.assertEqual(names[:4], ['pos', 'quat', 'axisangle', 'euler'])
    self.assertIn('friction', names)

  def test_attr_types_and_arity(self):
    schema = self.parse(GOOD)
    attrs = {a.name: a for a in schema.expanded_attrs(schema.elements['geom'])}
    self.assertEqual(attrs['condim'].arity, mjcf_schema.Arity(1, 1))
    self.assertEqual(attrs['quat'].arity, mjcf_schema.Arity(4, 4))
    self.assertEqual(attrs['size'].arity, mjcf_schema.Arity(0, 3))
    self.assertEqual(attrs['friction'].arity, mjcf_schema.Arity(1, 3))
    self.assertEqual(attrs['solref'].arity, mjcf_schema.Arity(0, 'mjNREF'))
    self.assertEqual(attrs['user'].arity, mjcf_schema.Arity(0, None))
    self.assertEqual(attrs['type'].type, 'enum')
    self.assertEqual(attrs['type'].target, 'geomtype')
    self.assertEqual(attrs['class'].type, 'ref')
    self.assertEqual(attrs['class'].target, 'defaults')
    self.assertEqual(attrs['name'].type, 'id')
    self.assertEqual(attrs['name'].target, 'geom')

  def test_defaults(self):
    schema = self.parse(GOOD)
    attrs = {a.name: a for a in schema.expanded_attrs(schema.elements['geom'])}
    self.assertEqual(attrs['friction'].default, (1, 0.005, 0.0001))
    self.assertEqual(attrs['condim'].default, 3.0)
    self.assertEqual(attrs['type'].default, 'sphere')
    self.assertEqual(attrs['eulerseq'].default, 'xyz')
    self.assertIsNone(attrs['size'].default)

  def test_facets(self):
    schema = self.parse(GOOD)
    attrs = {a.name: a for a in schema.expanded_attrs(schema.elements['geom'])}
    self.assertEqual(attrs['class'].facets, {'field': 'classname'})
    self.assertEqual(attrs['eulerseq'].facets, {'pattern': '[xyzXYZ]{3}'})
    self.assertEqual(attrs['margin'].facets, {'nodefault': True})
    self.assertEqual(attrs['file'].facets, {'required': True})

  def test_children(self):
    schema = self.parse(GOOD)
    children = schema.elements['geom'].children()
    self.assertEqual([(c.name, c.card) for c in children],
                     [('geom', '*'), ('defaults', 'R')])

  def test_docs(self):
    schema = self.parse(GOOD)
    self.assertEqual(schema.elements['geom'].doc, 'geometric entity')
    attrs = {a.name: a for a in schema.expanded_attrs(schema.elements['geom'])}
    self.assertEqual(attrs['friction'].doc, 'slide, roll, spin')
    self.assertEqual(attrs['axisangle'].doc, '(x, y, z, angle)')
    self.assertIsNone(attrs['condim'].doc)


class ErrorTest(googletest.TestCase):

  def error(self, text):
    with self.assertRaises(mjcf_schema.SchemaError) as ctx:
      mjcf_schema.parse_string(text)
    return str(ctx.exception)

  def test_error_has_line_number(self):
    message = self.error('element geom {\n  size ; double\n}')
    self.assertIn('<string>:2:', message)

  def test_duplicate_element(self):
    message = self.error('element geom {}\nelement geom {}')
    self.assertIn('duplicate element', message)

  def test_duplicate_attr(self):
    message = self.error('element geom {\n  a : int\n  a : double\n}')
    self.assertIn("duplicate attribute 'a'", message)

  def test_duplicate_attr_via_use(self):
    message = self.error('group g {\n  a : int\n}\n'
                         'element geom {\n  use g\n  a : double\n}')
    self.assertIn("duplicate attribute 'a'", message)

  def test_dangling_enum(self):
    message = self.error('element geom {\n  type : enum<nosuch>\n}')
    self.assertIn("undeclared enum 'nosuch'", message)

  def test_dangling_ref(self):
    message = self.error('element geom {\n  mesh : ref<nosuch>\n}')
    self.assertIn("namespace 'nosuch'", message)

  def test_ref_resolved_by_id_elsewhere(self):
    mjcf_schema.parse_string(
        'element mesh {\n  name : id<mesh>\n}\n'
        'element geom {\n  mesh : ref<mesh>\n}')

  def test_id_with_default(self):
    message = self.error('element geom {\n  name : id<geom> = "x"\n}')
    self.assertIn('may not have a default', message)

  def test_dangling_use(self):
    message = self.error('element geom {\n  use nosuch\n}')
    self.assertIn("undeclared group 'nosuch'", message)

  def test_dangling_child(self):
    message = self.error('element geom {\n  child nosuch *\n}')
    self.assertIn("undeclared element 'nosuch'", message)

  def test_use_cycle(self):
    message = self.error('group a {\n  use b\n}\ngroup b {\n  use a\n}')
    self.assertIn('cycle', message)

  def test_default_too_long(self):
    message = self.error('element geom {\n  size : double[3] = {1, 2, 3, 4}\n}')
    self.assertIn('at most 3', message)

  def test_default_too_short(self):
    message = self.error('element geom {\n  size : double[3] = {1, 2}\n}')
    self.assertIn('at least 3', message)

  def test_vector_default_on_scalar(self):
    message = self.error('element geom {\n  mass : double = {1, 2}\n}')
    self.assertIn('vector default for scalar', message)

  def test_enum_default_not_a_keyword(self):
    message = self.error('enum e {\n  a = 0\n}\n'
                         'element geom {\n  t : enum<e> = b\n}')
    self.assertIn('not a keyword', message)

  def test_unknown_facet(self):
    message = self.error('element geom {\n  a : int (frobnicate)\n}')
    self.assertIn("unknown facet 'frobnicate'", message)

  def test_required_with_default(self):
    message = self.error('element geom {\n  a : int = 1 (required)\n}')
    self.assertIn('required and has a default', message)

  def test_variant_with_required(self):
    message = self.error('group g variant {\n  a : int (required)\n}\n'
                         'element geom {\n  use g\n}')
    self.assertIn('may not be required', message)

  def test_variant_with_use(self):
    message = self.error('group inner {\n  a : int\n}\n'
                         'group g variant {\n  use inner\n}')
    self.assertIn("may not contain 'use'", message)

  def test_duplicate_enum_keyword(self):
    message = self.error('enum e {\n  a = 0\n  a = 1\n}')
    self.assertIn('duplicate enum keyword', message)

  def test_duplicate_child(self):
    message = self.error('element a {}\n'
                         'element geom {\n  child a *\n  child a ?\n}')
    self.assertIn("duplicate child 'a'", message)

  def test_empty_enum(self):
    message = self.error('enum e {\n}')
    self.assertIn('is empty', message)

  def test_child_in_group(self):
    message = self.error('group g {\n  child geom *\n}')
    self.assertIn('not allowed in a group', message)

  def test_decreasing_arity(self):
    message = self.error('element geom {\n  a : double[3..2]\n}')
    self.assertIn('not increasing', message)

  def test_constraints(self):
    schema = mjcf_schema.parse_string(
        'element connect {\n'
        '  site1 : ref<site>\n  site2 : ref<site>\n'
        '  body1 : string\n  anchor : double[3]\n'
        '  exclusive site1+site2 body1+anchor   # semantics cannot mix\n'
        '  oneof site1+site2 body1+anchor\n'
        '  requires site1 site2\n'
        '}\n'
        'element site {\n  name : id<site>\n}')
    cons = schema.elements['connect'].constraints()
    self.assertEqual([c.kind for c in cons],
                     ['exclusive', 'oneof', 'requires'])
    self.assertEqual(cons[0].bundles,
                     [('site1', 'site2'), ('body1', 'anchor')])
    self.assertEqual(cons[0].doc, 'semantics cannot mix')
    self.assertEqual(cons[2].bundles, [('site1',), ('site2',)])

  def test_constraint_unknown_attr(self):
    message = self.error('element a {\n  x : int\n  exclusive x nosuch\n}')
    self.assertIn("unknown attribute 'nosuch'", message)

  def test_requires_arity(self):
    message = self.error(
        'element a {\n  x : int\n  y : int\n  z : int\n'
        '  requires x y+z\n}')
    self.assertIn('exactly two attributes', message)

  def test_flags_type(self):
    schema = mjcf_schema.parse_string(
        'enum camout : mjtCamOutBit {\n  rgb = mjCAMOUT_RGB\n}\n'
        'element camera {\n  output : flags<camout>\n}')
    attr = schema.elements['camera'].members[0]
    self.assertEqual(attr.type, 'flags')
    self.assertEqual(attr.target, 'camout')

  def test_min_max_positive_facets(self):
    schema = mjcf_schema.parse_string(
        'element size {\n'
        '  nkey  : int (min=-1)\n'
        '  group : int (min=0, max=5)\n'
        '  znear : float (positive)\n'
        '}')
    attrs = {a.name: a for a in schema.elements['size'].members}
    self.assertEqual(attrs['nkey'].facets['min'], -1.0)
    self.assertEqual(attrs['group'].facets['max'], 5.0)
    self.assertTrue(attrs['znear'].facets['positive'])

  def test_min_on_string_rejected(self):
    message = self.error('element a {\n  s : string (min=0)\n}')
    self.assertIn('requires a numeric attribute', message)

  def test_const_member(self):
    schema = mjcf_schema.parse_string(
        'element touch : mjsSensor {\n'
        '  set type    = mjSENS_TOUCH   # sensor type from tag\n'
        '  set objtype = mjOBJ_SITE\n'
        '  a : int\n}')
    consts = schema.elements['touch'].consts()
    self.assertEqual([(c.field, c.value) for c in consts],
                     [('type', 'mjSENS_TOUCH'), ('objtype', 'mjOBJ_SITE')])
    self.assertEqual(consts[0].doc, 'sensor type from tag')

  def test_const_in_group_rejected(self):
    message = self.error('group g {\n  set type = mjSENS_TOUCH\n}')
    self.assertIn('not allowed in a group', message)

  def test_element_facets(self):
    schema = mjcf_schema.parse_string(
        'element body {}\n'
        'element eq_joint : mjsEquality (xml=joint) {\n'
        '  polycoef : double[5]\n}\n'
        'element frame (alias=body) {}\n')
    self.assertEqual(schema.elements['eq_joint'].xml_name(), 'joint')
    self.assertEqual(schema.elements['body'].xml_name(), 'body')
    self.assertEqual(schema.elements['frame'].facets, {'alias': 'body'})

  def test_element_field_facet(self):
    schema = mjcf_schema.parse_string(
        'element global : mjVisual (field=global) {\n  fovy : double\n}')
    self.assertEqual(schema.elements['global'].facets['field'], 'global')

  def test_element_unknown_facet(self):
    message = self.error('element geom (required) {}')
    self.assertIn("unknown facet 'required'", message)

  def test_element_dangling_alias(self):
    message = self.error('element frame (alias=nosuch) {}')
    self.assertIn("undeclared element 'nosuch'", message)

  def test_bool_type(self):
    schema = mjcf_schema.parse_string(
        'element compiler {\n  autolimits : bool = true\n}')
    attr = schema.elements['compiler'].members[0]
    self.assertEqual(attr.type, 'bool')
    self.assertEqual(attr.default, 'true')

  def test_bool_bad_default(self):
    message = self.error('element compiler {\n  autolimits : bool = maybe\n}')
    self.assertIn('must be true or false', message)

  def test_bool_vector_rejected(self):
    message = self.error('element compiler {\n  a : bool[2]\n}')
    self.assertIn('may not be a vector', message)

  def test_file_type(self):
    schema = mjcf_schema.parse_string(
        'element mesh {\n  file : file (required)  # mesh file\n}')
    attr = schema.elements['mesh'].members[0]
    self.assertEqual(attr.type, 'file')
    self.assertTrue(attr.arity.is_scalar())

  def test_file_vector_rejected(self):
    message = self.error('element mesh {\n  file : file[3]\n}')
    self.assertIn('may not be a vector', message)

  def test_pattern_on_numeric(self):
    message = self.error('element geom {\n  a : int (pattern="x")\n}')
    self.assertIn("'pattern' requires a text attribute", message)

  def test_chars_type(self):
    schema = mjcf_schema.parse_string(
        'element compiler {\n  eulerseq : chars[3] (pattern="[xyz]{3}")\n}')
    attr = schema.elements['compiler'].members[0]
    self.assertEqual(attr.type, 'chars')
    self.assertEqual((attr.arity.lo, attr.arity.hi), (3, 3))

  def test_chars_unbounded_rejected(self):
    message = self.error('element compiler {\n  a : chars[]\n}')
    self.assertIn('must declare a bounded length', message)

  def test_chars_with_default_rejected(self):
    message = self.error('element compiler {\n  a : chars[3] = "xyz"\n}')
    self.assertIn('may not have a default', message)

  def test_ref_with_default(self):
    message = self.error('element a {\n  name : id<a>\n}\n'
                         'element geom {\n  r : ref<a> = a\n}')
    self.assertIn('may not have a default', message)

  def test_string_lexing_range_vs_float(self):
    # 0..3 must lex as a range, not the floats '0.' and '.3'.
    schema = mjcf_schema.parse_string(
        'element geom {\n  a : double[0..3] = {0.5, .25, 1e-3}\n}')
    attr = schema.elements['geom'].members[0]
    self.assertEqual(attr.arity, mjcf_schema.Arity(0, 3))
    self.assertEqual(attr.default, (0.5, 0.25, 0.001))

  def test_min_max_integer_facet(self):
    # Integer values in facets dictionary must be accepted as numeric.
    attr = mjcf_schema.Attr(
        name='group', type='int', target=None,
        arity=mjcf_schema.Arity(1, 1), default=None,
        facets={'min': 0, 'max': 5}, doc=None, line=1)
    element = mjcf_schema.Element(
        name='geom', spec=None, facets={}, members=[attr], doc=None, line=1)
    schema = mjcf_schema.Schema(
        enums={}, groups={}, elements={'geom': element}, path='<test>')
    mjcf_schema._validate(schema)

  def test_min_greater_than_max(self):
    message = self.error(
        'element geom {\n  a : double (min=10, max=5)\n}')
    self.assertIn("facet 'min' cannot be greater than 'max'", message)


if __name__ == '__main__':
  googletest.main()
