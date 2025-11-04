# Copyright 2025 DeepMind Technologies Limited
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

from absl.testing import absltest
from introspect import ast_nodes
from wasm.codegen.helpers import struct_constructor_code_builder
from wasm.codegen.helpers import struct_field_handler
from wasm.codegen.helpers import structs_parser


class StructConstructorCodeBuilderTest(absltest.TestCase):

  def test_constructor_code_with_default_function(self):
    wrapped_structs = structs_parser.generate_wasm_bindings(["mjLROpt"])
    self.assertEqual(
        wrapped_structs["mjLROpt"].wrapped_source,
        """
MjLROpt::MjLROpt(mjLROpt *ptr) : ptr_(ptr) {}
MjLROpt::MjLROpt() : ptr_(new mjLROpt) {
  owned_ = true;
  mj_defaultLROpt(ptr_);
}
MjLROpt::MjLROpt(const MjLROpt &other) : MjLROpt() {
  *ptr_ = *other.get();
}
MjLROpt& MjLROpt::operator=(const MjLROpt &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjLROpt::~MjLROpt() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjLROpt> MjLROpt::copy() {
  return std::make_unique<MjLROpt>(*this);
}
""".strip(),
    )

  def test_constructor_code_without_default_function(self):
    wrapped_structs = structs_parser.generate_wasm_bindings(["mjsElement"])
    self.assertEqual(
        wrapped_structs["mjsElement"].wrapped_source,
        """
MjsElement::MjsElement(mjsElement *ptr) : ptr_(ptr) {}
MjsElement::~MjsElement() {}
std::unique_ptr<MjsElement> MjsElement::copy() {
  return std::make_unique<MjsElement>(*this);
}
""".strip(),
    )

  def test_constructor_code_with_fields_with_init(self):
    field_with_init = ast_nodes.StructFieldDecl(
        name="element",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="mjsElement"),
        ),
        doc="",
    )
    wrapped_field_data = struct_field_handler.StructFieldHandler(
        field_with_init, "MjsTexture"
    ).generate()
    self.assertEqual(
        struct_constructor_code_builder.build_struct_source(
            "mjsTexture",
            [wrapped_field_data],
        ),
        """
MjsTexture::MjsTexture(mjsTexture *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsTexture::~MjsTexture() {}
""".strip(),
    )

  def test_constructor_code_with_shallow_copy(self):
    self.assertEqual(
        struct_constructor_code_builder.build_struct_source("mjvLight", []),
        """MjvLight::MjvLight(mjvLight *ptr) : ptr_(ptr) {}
MjvLight::MjvLight() : ptr_(new mjvLight) {
  owned_ = true;
}
MjvLight::MjvLight(const MjvLight &other) : MjvLight() {
  *ptr_ = *other.get();
}
MjvLight& MjvLight::operator=(const MjvLight &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjvLight::~MjvLight() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjvLight> MjvLight::copy() {
  return std::make_unique<MjvLight>(*this);
}""".strip(),
    )


def test_build_struct_header_with_nested_wrappers(self):
  self.assertEqual(
      struct_constructor_code_builder.build_struct_header("mjData", []),
      "",
  )


def test_build_struct_header_basic_struct(self):
  self.assertEqual(
      struct_constructor_code_builder.build_struct_header("mjLROpt", []),
      """
struct MjLROpt {
  MjLROpt();
  MjLROpt(const MjLROpt &);
  MjLROpt &operator=(const MjLROpt &);
  explicit MjLROpt(mjLROpt *ptr);
  ~MjLROpt();
  mjLROpt* get() const { return ptr_; }
  void set(mjLROpt* ptr) { ptr_ = ptr; }

 private:
  mjLROpt* ptr_;
  bool owned_ = false;
};
""".strip(),
  )


if __name__ == "__main__":
  absltest.main()
