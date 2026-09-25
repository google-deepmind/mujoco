"""Compare simulation states and timings from matched build configurations."""

import argparse
import json
import math
import statistics
import subprocess


def run(executable, model, steps):
  result = subprocess.run(
      [executable, model, str(steps)], check=True, capture_output=True, text=True
  )
  return json.loads(result.stdout)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--bazel", required=True)
  parser.add_argument("--cmake", required=True)
  parser.add_argument("--steps", type=int, default=256)
  parser.add_argument("--repetitions", type=int, default=7)
  parser.add_argument("--atol", type=float, default=0.0)
  parser.add_argument("--rtol", type=float, default=0.0)
  parser.add_argument("models", nargs="+")
  args = parser.parse_args()
  if args.repetitions < 2 or args.steps < 1:
    parser.error("require positive steps and repeated measurements")
  failed = False
  reports = []
  for model in args.models:
    measurements = {"bazel": [], "cmake": []}
    states = {}
    for repeat in range(args.repetitions):
      order = ["bazel", "cmake"] if repeat % 2 else ["cmake", "bazel"]
      for build in order:
        result = run(getattr(args, build), model, args.steps)
        measurements[build].append(result.pop("seconds"))
        if build in states and result != states[build]:
          raise RuntimeError(f"Nonrepeatable simulation: {build}, {model}")
        states[build] = result
    left, right = states["bazel"], states["cmake"]
    equal = all(left[key] == right[key] for key in ["version", "nq", "nv", "contacts"])
    equal &= len(left["state"]) == len(right["state"])
    equal &= all(
        math.isfinite(a) and math.isfinite(b)
        and math.isclose(a, b, rel_tol=args.rtol, abs_tol=args.atol)
        for a, b in zip(left["state"], right["state"])
    )
    failed |= not equal
    reports.append({
        "model": model,
        "states_match": equal,
        "seconds": measurements,
        "bazel_over_cmake_median": statistics.median(measurements["bazel"])
        / statistics.median(measurements["cmake"]),
    })
  print(json.dumps(reports, indent=2))
  raise SystemExit(int(failed))


if __name__ == "__main__":
  main()
