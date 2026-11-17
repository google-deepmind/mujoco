#!/bin/bash
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

if [ -n "${TEST_SRCDIR:-}" ]; then
  readonly MODEL="${TEST_SRCDIR}/${TEST_WORKSPACE}/model/sleep/dominos.xml"
  readonly TARGET_BINARY="${TEST_SRCDIR}/${TEST_WORKSPACE}/sample/testspeed"
else
  MODEL="${CMAKE_SOURCE_DIR}/model/sleep/dominos.xml"
fi

die() { echo "$*" 1>&2 ; exit 1; }

if [ -z "$TARGET_BINARY" ]; then
  die "Expecting environment variable TARGET_BINARY."
fi

if [ -n "$MUJOCO_DLL_DIR" ]; then
  # Extend PATH to include the directory containing the mujoco DLL.
  # This is needed on Windows.
  PATH=$PATH:$MUJOCO_DLL_DIR
fi

# Run testspeed with override flags and verify they get applied
echo "Running testspeed with command-line overrides on dominos.xml..."
echo "Command: $TARGET_BINARY --nstep=10 --nthread=1 --noisestd=0.02 --noiserate=0.2 --solver=PGS --jacobian=sparse --integrator=rk4 --iterations=12 --tolerance=1e-7 --sleep_tolerance=2e-4 --noslip_iterations=5 $MODEL"

OUTPUT=$("$TARGET_BINARY" --nstep=10 --nthread=1 --noisestd=0.02 --noiserate=0.2 --solver=PGS --jacobian=sparse --integrator=rk4 --iterations=12 --tolerance=1e-7 --sleep_tolerance=2e-4 --noslip_iterations=5 "$MODEL") || die "testspeed failed"

# Verify option printing works.
# Dominos.xml has:
#   cone="elliptic" -> cone: Elliptic
#   impratio="10" -> impratio: 10
#   <flag sleep="enable"/> -> Sleep: Enabled
#
# Overridden via command line options:
#   solver=PGS
#   jacobian=sparse -> jacobian: Sparse
#   integrator=rk4 -> integrator: RK4
#   iterations=12
#   tolerance=1e-7 -> tolerance: 1e-07
#   sleep_tolerance=2e-4 (overwrites XML's 3e-4) -> sleep_tolerance: 0.0002
#   noslip_iterations=5

echo "Full testspeed Output:"
echo "--------------------------------------------------------"
echo "$OUTPUT"
echo "--------------------------------------------------------"

echo "Verifying printed non-default option block:"
echo "$OUTPUT" | grep -A 10 "Physics options (non-default):" || die "Missing physics options block!"
echo "--------------------------------------------------------"

echo "$OUTPUT" | grep -F "Physics options (non-default):" || die "Missing non-default header"
echo "$OUTPUT" | grep -F "  cone              : Elliptic" || die "Missing cone print"
echo "$OUTPUT" | grep -F "  impratio          : 10" || die "Missing impratio print"
echo "$OUTPUT" | grep -E "  sleep_tolerance   : (0.0002|2e-0?4)" || die "Missing sleep_tolerance print"
echo "$OUTPUT" | grep -F "  Sleep             : Enabled" || die "Missing Sleep print"
echo "$OUTPUT" | grep -F "  solver            : PGS" || die "Missing solver print"
echo "$OUTPUT" | grep -F "  jacobian          : Sparse" || die "Missing jacobian print"
echo "$OUTPUT" | grep -F "  integrator        : RK4" || die "Missing integrator print"
echo "$OUTPUT" | grep -F "  iterations        : 12" || die "Missing iterations print"
echo "$OUTPUT" | grep -E "  tolerance         : (1e-0?7|0.0000001|1e-7)" || die "Missing tolerance print"
echo "$OUTPUT" | grep -F "  noslip_iterations : 5" || die "Missing noslip_iterations print"
if echo "$OUTPUT" | grep -Fq "worker thread"; then
  die "Unexpected worker thread banner with default --nenginethread=0"
fi
echo "$OUTPUT" | grep -F " Simulation time      :" || die "Missing Simulation time in text output"
echo "$OUTPUT" | grep -F " Steps per second     :" || die "Missing Steps per second in text output"
if echo "$OUTPUT" | grep -q '^{'; then
  die "Unexpected JSON output in default text mode"
fi

# Verify explicit --format=text outputs human-readable text, not JSON.
OUTPUT_TEXT=$("$TARGET_BINARY" --nstep=2 --format=text "$MODEL") || die "testspeed --format=text failed"
echo "$OUTPUT_TEXT" | grep -F " Steps per second     :" || die "Missing Steps per second in --format=text"
if echo "$OUTPUT_TEXT" | grep -q '^{'; then
  die "Unexpected JSON output in --format=text mode"
fi

# Verify --nenginethread worker thread banner for 1 and 2 worker threads.
OUTPUT_T1=$("$TARGET_BINARY" --nstep=2 --nenginethread=1 "$MODEL") || die "testspeed --nenginethread=1 failed"
echo "$OUTPUT_T1" | grep -F ", using 1 worker thread..." || die "Missing 1 worker thread banner"

OUTPUT_T2=$("$TARGET_BINARY" --nstep=2 --nenginethread=2 "$MODEL") || die "testspeed --nenginethread=2 failed"
echo "$OUTPUT_T2" | grep -F ", using 2 worker threads..." || die "Missing 2 worker threads banner"

# Verify --format=json output and timer component decomposition (other == step - sum(stages))
OUTPUT_JSON=$("$TARGET_BINARY" --nstep=20 --format=json "$MODEL") || die "testspeed --format=json failed"
echo "$OUTPUT_JSON" | grep -F '"steps_per_sec":' || die "Missing steps_per_sec in JSON"
echo "$OUTPUT_JSON" | grep -F '"us_per_step":' || die "Missing us_per_step in JSON"
echo "$OUTPUT_JSON" | grep -F '"iters_per_step":' || die "Missing iters_per_step in JSON"
echo "$OUTPUT_JSON" | grep -F '"ncon":' || die "Missing ncon in JSON"
echo "$OUTPUT_JSON" | grep -F '"nefc":' || die "Missing nefc in JSON"
echo "$OUTPUT_JSON" | grep -F '"nv":' || die "Missing nv in JSON"
echo "$OUTPUT_JSON" | grep -F '"pos_kinematics":' || die "Missing pos_kinematics in JSON"
echo "$OUTPUT_JSON" | grep -F '"col_narrowphase":' || die "Missing col_narrowphase in JSON"

PYTHON_BIN="$(command -v python3 || command -v python)" || die "Python interpreter not found"
OUTPUT_JSON_RK4=$("$TARGET_BINARY" --nstep=2 --integrator=rk4 --format=json "$MODEL") || die "testspeed --integrator=rk4 --format=json failed"

OUTPUT_JSON="$OUTPUT_JSON" OUTPUT_JSON_RK4="$OUTPUT_JSON_RK4" "$PYTHON_BIN" - <<'PY' || die "JSON timer consistency check failed"
import json
import math
import os

def is_finite(x):
  return isinstance(x, (int, float)) and not math.isnan(x) and not math.isinf(x)

def reject_nonfinite(c):
  raise ValueError("Non-finite JSON constant: %s" % (c,))

stages = ["position", "velocity", "actuation", "constraint", "advance"]

for label, env_key, expect_advance in [
    ("euler", "OUTPUT_JSON", True),
    ("rk4", "OUTPUT_JSON_RK4", False),
]:
  data = json.loads(os.environ[env_key], parse_constant=reject_nonfinite)
  t = data["timers"]
  assert is_finite(t["step"]) and t["step"] > 0, "%s: bad step=%r" % (label, t["step"])
  for s in stages:
    assert is_finite(t[s]) and t[s] >= 0, "%s: bad %s=%r" % (label, s, t[s])
  if expect_advance:
    assert t["advance"] > 0, "euler: expected advance > 0, got %r" % (t["advance"],)
  else:
    assert t["advance"] == 0, "rk4: expected advance == 0 (num=0), got %r" % (t["advance"],)
  comp = sum(t[s] for s in stages)
  assert comp > 0, "%s: expected comp > 0, got %r" % (label, comp)
  assert is_finite(t["other"]) and t["other"] < t["step"], (
      "%s: bad other=%r step=%r" % (label, t["other"], t["step"])
  )
  tol = 1e-4 * t["step"] + 1e-6
  diff = abs(t["other"] - (t["step"] - comp))
  assert diff <= tol, "%s: other=%r != step-comp=%r (diff=%r, tol=%r)" % (
      label, t["other"], t["step"] - comp, diff, tol
  )
PY

# Verify --ctrl trajectory replay, default nstep, last-row clamping, and validation
WORK_DIR="${TEST_TMPDIR:-$(mktemp -d)}"
CTRL_MODEL="${WORK_DIR}/ctrl_model.xml"
CTRL_FILE="${WORK_DIR}/ctrl_valid.txt"
CTRL_CLAMP="${WORK_DIR}/ctrl_clamp.txt"
CTRL_BAD="${WORK_DIR}/ctrl_bad_cols.txt"

cat > "$CTRL_MODEL" <<'EOF'
<mujoco>
  <option timestep="0.01" gravity="0 0 0"/>
  <worldbody>
    <geom type="plane" size="2 2 0.01"/>
    <body pos="0 0 0.12">
      <joint name="jx" type="slide" axis="1 0 0"/>
      <joint name="jz" type="slide" axis="0 0 1"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="jx"/>
    <motor joint="jz"/>
  </actuator>
</mujoco>
EOF

cat > "$CTRL_FILE" <<'EOF'
# comment header and blank lines should be skipped

0.1 0.2
0.3, -0.4  # inline comment
-0.5,0.6
0.0 -1.0
EOF

# 1) Default nstep inferred from row count (4 rows)
OUTPUT_CTRL=$("$TARGET_BINARY" --ctrl="$CTRL_FILE" "$CTRL_MODEL") || die "testspeed --ctrl default nstep failed"
echo "$OUTPUT_CTRL" | grep -F "Rolling out 4 steps at dt = 0.01" || die "Expected default nstep=4 from --ctrl row count"

# 2) Explicit --nstep > nrows clamps last row (row 1 pushes up +z, row 2 pushes down -z into plane -> contacts > 0)
cat > "$CTRL_CLAMP" <<'EOF'
0.0 5.0
0.0 -20.0
EOF
OUTPUT_CLAMP=$("$TARGET_BINARY" --ctrl="$CTRL_CLAMP" --nstep=30 --format=json "$CTRL_MODEL") || die "testspeed --ctrl clamp failed"
if echo "$OUTPUT_CLAMP" | grep -Eq '"ncon": 0([,} ]|$)'; then
  die "Expected clamped downward control to produce contacts, got: $OUTPUT_CLAMP"
fi

# 3) Error on wrong column count (3 columns for nu=2)
echo "0.1 0.2 0.3" > "$CTRL_BAD"
if "$TARGET_BINARY" --ctrl="$CTRL_BAD" "$CTRL_MODEL" >/dev/null 2>&1; then
  die "Expected non-zero exit on wrong column count in --ctrl"
fi

# 4) Error when applying non-empty --ctrl to nu=0 model (dominos.xml)
if "$TARGET_BINARY" --ctrl="$CTRL_FILE" "$MODEL" >/dev/null 2>&1; then
  die "Expected non-zero exit when passing non-empty --ctrl to nu=0 model"
fi

echo "PASS"
