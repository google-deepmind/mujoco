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
# ==============================================================================

set -euo pipefail

# --- Logging helpers ---------------------------------------------------------
log_stage() { echo -e "\n\033[1;34m==== $1 ====\033[0m"; }
log_ok()    { echo -e "\033[1;32m  ✓ $1\033[0m"; }
log_fail()  { echo -e "\033[1;31m  ✗ $1\033[0m"; }

run_shim() {
  local label="$1"; shift
  log_stage "Generating shim: ${label}"
  echo "  → $*"
  local output
  if output=$("$@" --logtostderr 2>&1); then
    # Show only Python-level log lines (filters noisy C++ infra logs).
    echo "$output" | grep -E '\.py' || true
    log_ok "${label}"
  else
    echo "$output"
    log_fail "${label}"
    exit 1
  fi
}

# --- Path setup --------------------------------------------------------------
mjwarp_base="mujoco/mjx/third_party/mujoco_warp/_src"
mjx_base="mujoco/mjx"

# Derived paths (shared).
mjwarp="${mjwarp_base}"
mjx_warp_out="${mjx_base}/warp"
mjx_types="${mjx_base}/_src/types.py"
mjx_warp_types="${mjx_base}/warp/types.py"

log_stage "Path configuration"
echo "  mjwarp_base  = ${mjwarp_base}"
echo "  mjx_base     = ${mjx_base}"
echo "  mjx_types    = ${mjx_types}"
echo "  output dir   = ${mjx_warp_out}"

# --- Stage 1: Generate warp types -------------------------------------------
log_stage "Stage 1/3: Generating warp types"
python mujoco/mjx/codegen/generate_warp_types.py \
  --mjx_warp_types_out_path=${mjx_warp_types} \
  --mjx_types_path=${mjx_types}
log_ok "Warp types written to ${mjx_warp_types}"

# --- Stage 2: Build shim generator ------------------------------------------
log_stage "Stage 2/3: Building shim generator"
generate_warp_shim="python mujoco/mjx/codegen/generate_warp_shim.py"
log_ok "Shim generator ready"

# --- Stage 3: Generate shim code for each function --------------------------
log_stage "Stage 3/3: Generating shim code"

# Smooth.
run_shim "smooth:kinematics" ${generate_warp_shim} \
 --mjwarp_function=${mjwarp}/smooth.py:kinematics \
 --mjx_warp_output_path=${mjx_warp_out}/ \
 --mjwarp_types=${mjwarp}/types.py

run_shim "smooth:tendon" ${generate_warp_shim} \
 --mjwarp_function=${mjwarp}/smooth.py:tendon \
 --mjx_warp_output_path=${mjx_warp_out}/ \
 --mjwarp_types=${mjwarp}/types.py \
 --append_to_output_file=True

run_shim "smooth:com_pos" ${generate_warp_shim} \
 --mjwarp_function=${mjwarp}/smooth.py:com_pos \
 --mjx_warp_output_path=${mjx_warp_out}/ \
 --mjwarp_types=${mjwarp}/types.py \
 --append_to_output_file=True

# Collision.
run_shim "collision_driver:collision" ${generate_warp_shim} \
 --mjwarp_function=${mjwarp}/collision_driver.py:collision \
 --mjx_warp_output_path=${mjx_warp_out}/ \
 --mjwarp_types=${mjwarp}/types.py

# Forward.
run_shim "forward:forward" ${generate_warp_shim} \
 --mjwarp_function=${mjwarp}/forward.py:forward \
 --mjx_warp_output_path=${mjx_warp_out}/ \
 --mjwarp_types=${mjwarp}/types.py

run_shim "forward:step" ${generate_warp_shim} \
 --mjwarp_function=${mjwarp}/forward.py:step \
 --mjx_warp_output_path=${mjx_warp_out}/ \
 --mjwarp_types=${mjwarp}/types.py \
 --append_to_output_file=True

# Render and bvh.
run_shim "bvh:refit_bvh" ${generate_warp_shim} \
 --mjwarp_function=${mjwarp}/bvh.py:refit_bvh \
 --mjx_warp_output_path=${mjx_warp_out}/ \
 --mjwarp_types=${mjwarp}/types.py

run_shim "render:render" ${generate_warp_shim} \
 --mjwarp_function=${mjwarp}/render.py:render \
 --mjx_warp_output_path=${mjx_warp_out}/ \
 --mjwarp_types=${mjwarp}/types.py

log_stage "Done"
log_ok "All shims generated successfully"
