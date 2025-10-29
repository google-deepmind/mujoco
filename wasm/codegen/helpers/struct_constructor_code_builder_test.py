import unittest
from introspect import ast_nodes
from wasm.codegen.helpers import struct_constructor_code_builder
from wasm.codegen.helpers import struct_field_handler

StructFieldDecl = ast_nodes.StructFieldDecl
ValueType = ast_nodes.ValueType
PointerType = ast_nodes.PointerType
ArrayType = ast_nodes.ArrayType
StructDecl = ast_nodes.StructDecl


class StructConstructorCodeBuilderTest(unittest.TestCase):

  def test_constructor_code_with_default_function(self):
    self.assertEqual(
        struct_constructor_code_builder.build_struct_source(
            "mjLROpt", "mj_defaultLROpt"),
        """
MjLROpt::MjLROpt(mjLROpt *ptr) : ptr_(ptr) {}
MjLROpt::MjLROpt() : ptr_(new mjLROpt) {
  owned_ = true;
  mj_defaultLROpt(ptr_);
}
MjLROpt::~MjLROpt() {
  if (owned_ && ptr_) delete ptr_;
}
""".strip(),
    )

  def test_constructor_code_without_default_function(self):
    self.assertEqual(
        struct_constructor_code_builder.build_struct_source("mjLROpt"),
        """
MjLROpt::MjLROpt(mjLROpt *ptr) : ptr_(ptr) {}
MjLROpt::MjLROpt() : ptr_(new mjLROpt) {
  owned_ = true;
}
MjLROpt::~MjLROpt() {
  if (owned_ && ptr_) delete ptr_;
}
""".strip(),
    )

  def test_constructor_code_with_fields_with_init(self):
    field_with_init = StructFieldDecl(
        name="element",
        type=PointerType(inner_type=ValueType(name="mjsElement"), ),
        doc="",
    )
    wrapped_field_data = struct_field_handler.StructFieldHandler(
        field_with_init, "MjsTexture").generate()
    self.assertEqual(
        struct_constructor_code_builder.build_struct_source(
            "mjsTexture",
            "mjs_defaultTexture",
            [wrapped_field_data],
        ),
        """
MjsTexture::MjsTexture(mjsTexture *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsTexture::~MjsTexture() {}
""".strip(),
    )

  def test_constructor_code_with_shallow_copy(self):
    self.assertEqual(
        struct_constructor_code_builder.build_struct_source(
            "mjvLight", use_shallow_copy=True),
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
        struct_constructor_code_builder.build_struct_header("mjData"),
        "",
    )

  def test_build_struct_header_basic_struct(self):
    self.assertEqual(
        struct_constructor_code_builder.build_struct_header(
            struct_name="mjLROpt", fields_with_init=[], wrapped_fields=[]),
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
  unittest.main()
