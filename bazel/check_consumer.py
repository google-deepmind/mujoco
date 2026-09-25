"""Exercise MuJoCo from a separate Bzlmod root without repository settings."""

import argparse
import hashlib
import json
import pathlib
import os
import base64
import subprocess
import tarfile
import tempfile


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--source", type=pathlib.Path, default=pathlib.Path(__file__).resolve().parents[1])
  parser.add_argument("--bazel", default="bazel")
  parser.add_argument("--archive", action="store_true")
  parser.add_argument("--output-user-root", type=pathlib.Path)
  args = parser.parse_args()
  source = args.source.resolve()
  with tempfile.TemporaryDirectory(prefix="mujoco-consumer-") as directory:
    root = pathlib.Path(directory)
    (root / ".bazelversion").write_text((source / ".bazelversion").read_text())
    module = ('module(name = "mujoco_consumer")\n'
              'bazel_dep(name = "mujoco", version = "3.14.1")\n'
              'bazel_dep(name = "rules_cc", version = "0.2.25")\n')
    if args.archive:
      archive = root / "mujoco.tar.gz"
      tracked = subprocess.check_output(
          ["git", "ls-files", "-z"], cwd=source
      ).decode().split("\0")
      files = {name for name in tracked if name and (source / name).is_file()}
      for directory, directories, names in os.walk(source):
        directories[:] = [name for name in directories if name not in {".git", ".cache", "node_modules"} and not name.startswith(("bazel-", "build"))]
        for name in names:
          if name == "BUILD.bazel" or name.endswith(".bzl"):
            files.add(str((pathlib.Path(directory) / name).relative_to(source)))
      files.update(str(p.relative_to(source)) for p in (source / "bazel").rglob("*") if p.is_file() and "__pycache__" not in p.parts)
      files.update(["MODULE.bazel", "MODULE.bazel.lock", ".bazelignore"])
      with tarfile.open(archive, "w:gz") as output:
        for name in sorted(files):
          output.add(source / name, arcname=name, recursive=False)
      digest = hashlib.sha256(archive.read_bytes()).hexdigest()
      module += f'archive_override(module_name = "mujoco", urls = [{json.dumps(archive.as_uri())}], integrity = "sha256-{base64.b64encode(bytes.fromhex(digest)).decode()}", strip_prefix = "")\n'
      print(f"Archive SHA-256: {digest}", flush=True)
    else:
      module += f'local_path_override(module_name = "mujoco", path = {json.dumps(str(source))})\n'
    (root / "MODULE.bazel").write_text(module)
    (root / "BUILD.bazel").write_text('''load("@rules_cc//cc:cc_test.bzl", "cc_test")
cc_test(name = "static", srcs = ["smoke.c"], deps = ["@mujoco//:mujoco"], linkstatic = True)
cc_test(name = "dynamic_deps", srcs = ["smoke.c"], deps = ["@mujoco//:mujoco"], linkstatic = False)
cc_test(name = "shared", srcs = ["smoke.c"], deps = ["@mujoco//:mujoco_shared"], linkstatic = True)
cc_test(name = "plugin", srcs = ["plugin.c"], deps = ["@mujoco//plugin/actuator:actuator"], linkstatic = True)
''')
    (root / "plugin.c").write_text('''#include <mujoco/mujoco.h>
int main(void) { return mjp_getPlugin("mujoco.pid", 0) == 0; }
''')
    (root / "smoke.c").write_text('''#include <mujoco/mujoco.h>
int main(void) {
  char error[1024] = {0};
  mjSpec* spec = mj_parseXMLString("<mujoco><worldbody><body><freejoint/><geom size='.1'/></body></worldbody></mujoco>", 0, error, sizeof(error));
  if (!spec) return 1;
  mjModel* model = mj_compile(spec, 0);
  if (!model) return 2;
  mjData* data = mj_makeData(model);
  mj_step(model, data);
  int failed = data->time <= 0 || model->nq != 7 || mj_version() != mjVERSION_HEADER;
  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
  return failed;
}
''')
    command = [args.bazel, "--ignore_all_rc_files"]
    if args.output_user_root:
      command += ["--output_user_root=" + str(args.output_user_root.resolve())]
    command += ["test", "//:static", "//:dynamic_deps", "//:shared", "//:plugin", "--jobs=8", "--repo_contents_cache=", "--test_output=errors"]
    subprocess.run(command, cwd=root, check=True)


if __name__ == "__main__":
  main()
