"""Generates TypeScript code that tests Mujoco enums."""

import os
import textwrap

from introspect import enums as introspect_enums
from codegen.helpers import common

write_to_file = common.write_to_file


def generate_typescript_enum_tests():
  """Generates TypeScript code that tests Mujoco enums."""
  output = textwrap.dedent("""\
    import 'jasmine';


    import { MainModule } from "../dist/mujoco_wasm"
    import loadMujoco from "../dist/mujoco_wasm.js"


    let mujoco: MainModule;


    describe('Enums', () => {
      beforeAll(async () => {
        mujoco = await loadMujoco();
      });
    """)

  for enum_decl in introspect_enums.ENUMS.values():
    enum_name = enum_decl.name
    output += f"""
  it('{enum_name} should exist', () => {{
    expect(mujoco.{enum_name}).toBeDefined();
  }});\n"""

  output += "});\n"
  return output


if __name__ == "__main__":
  ts_test_code = generate_typescript_enum_tests()
  output_file = './tests/enums_test.ts'
  write_to_file(output_file, ts_test_code)
