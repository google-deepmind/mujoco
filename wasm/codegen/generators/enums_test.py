import unittest

from introspect import ast_nodes

from generators import enums


class EnumsGeneratorTest(unittest.TestCase):

  def test_generate_enum_bindings(self):

    generator = enums.Generator({
        "TestEnum": ast_nodes.EnumDecl(
            name="TestEnum",
            declname="enum TestEnum_",
            values={"FIRST_VAL": 0, "SECOND_VAL": 1, "THIRD_VAL": 2},
        ),
        "AnotherEnum": ast_nodes.EnumDecl(
            name="AnotherEnum",
            declname="enum AnotherEnum_",
            values={"ALPHA": 100, "BETA": 200},
        ),
        "EmptyEnum": ast_nodes.EnumDecl(
            name="EmptyEnum",
            declname="enum EmptyEnum_",
            values={},
        ),
    })

    expected_code = """  enum_<TestEnum>("TestEnum")
    .value("FIRST_VAL", FIRST_VAL)
    .value("SECOND_VAL", SECOND_VAL)
    .value("THIRD_VAL", THIRD_VAL);

  enum_<AnotherEnum>("AnotherEnum")
    .value("ALPHA", ALPHA)
    .value("BETA", BETA);

  enum_<EmptyEnum>("EmptyEnum");
"""

    actual_code = generator.generate()

    self.assertEqual(actual_code, expected_code)


if __name__ == "__main__":
  unittest.main()
