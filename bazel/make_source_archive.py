"""Create the BCR source release asset from a committed MuJoCo revision."""

import argparse
import gzip
import pathlib
import re
import shutil
import subprocess
import tempfile


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--source", type=pathlib.Path,
      default=pathlib.Path(__file__).resolve().parents[1],
  )
  parser.add_argument("--revision", default="HEAD")
  parser.add_argument("--output", type=pathlib.Path, required=True)
  args = parser.parse_args()
  source = args.source.resolve()
  revision = subprocess.check_output(
      ["git", "rev-parse", "--verify", "--end-of-options", args.revision + "^{commit}"],
      cwd=source, text=True,
  ).strip()
  try:
    module = subprocess.check_output(
        ["git", "show", revision + ":MODULE.bazel"], cwd=source, text=True,
    )
  except subprocess.CalledProcessError:
    parser.error("the committed revision must contain MODULE.bazel")
  declaration = re.search(r"\bmodule\s*\((.*?)\)", module, re.DOTALL)
  version = (
      re.search(r'\bversion\s*=\s*[\"\']([\w.+-]+)[\"\']', declaration[1])
      if declaration else None
  )
  if not version:
    parser.error("MODULE.bazel must declare the module version")
  version = version[1]
  with tempfile.TemporaryFile() as archive:
    subprocess.run(
        ["git", "archive", "--format=tar", "--prefix=mujoco-" + version + "/", revision],
        cwd=source, stdout=archive, check=True,
    )
    archive.seek(0)
    with args.output.open("wb") as destination:
      with gzip.GzipFile(filename="", mode="wb", fileobj=destination, mtime=0) as compressed:
        shutil.copyfileobj(archive, compressed)
  print(f"{revision}: {args.output} (mujoco-{version}/)")


if __name__ == "__main__":
  main()
