"""Generates TypeScript code that tests Mujoco enums."""

import os
import textwrap

from mujoco.introspect import enums as introspect_enums

from google3.third_party.mujoco.wasm.codegen.helpers import common


write_to_file = common.write_to_file


def generate_typescript_enum_tests():
  """Generates TypeScript code that tests Mujoco enums."""
  output = textwrap.dedent("""\
    import 'jasmine';

    import {MainModule} from 'google3/third_party/mujoco/wasm/codegen/generated/bindings/mujoco';

    declare function loadMujoco(): Promise<MainModule>;

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
  output_file = os.path.join(
      os.getenv("BUILD_WORKSPACE_DIRECTORY"),
      "third_party/mujoco/wasm/tests/enums_test.ts",
  )
  write_to_file(output_file, ts_test_code)
