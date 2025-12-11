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

"""Constants used in the code generation process."""

from typing import Dict, Set
from introspect import structs as introspect_structs

PRIMITIVE_TYPES: Set[str] = {
    # go/keep-sorted start
    "char",
    "double",
    "float",
    "int",
    "mjtByte",
    "mjtMeshBuiltin",
    "mjtNum",
    "mjtObj",  # Adding this to the primitives because it is used as int,
    "mjtSize",
    "size_t",
    "uint64_t",
    "uintptr_t",
    "unsigned char",
    "unsigned int",
    "void",
    # go/keep-sorted end
}

_SKIPPED_PLUGIN_FUNCTIONS: tuple[str, ...] = (
    # go/keep-sorted start
    "mj_getPluginConfig",
    "mj_loadAllPluginLibraries",
    "mj_loadPluginLibrary",
    "mjc_distance",
    "mjc_getSDF",
    "mjc_gradient",
    "mjp_defaultDecoder",
    "mjp_defaultPlugin",
    "mjp_defaultResourceProvider",
    "mjp_findDecoder",
    "mjp_getPlugin",
    "mjp_getPluginAtSlot",
    "mjp_getResourceProvider",
    "mjp_getResourceProviderAtSlot",
    "mjp_pluginCount",
    "mjp_registerDecoder",
    "mjp_registerPlugin",
    "mjp_registerResourceProvider",
    "mjp_resourceProviderCount",
    # go/keep-sorted end
)

# Functions that are bound as class methods
_SKIPPED_CLASS_METHODS: tuple[str, ...] = (
    # go/keep-sorted start
    "mj_compile",
    "mj_copyData",
    "mj_copyModel",
    "mj_copySpec",
    "mj_deleteData",
    "mj_deleteModel",
    "mj_deleteSpec",
    "mj_loadXML",
    "mj_makeData",
    "mj_makeSpec",
    "mj_parse",  # TODO(manevi): Bind this function.
    "mj_parseXML",  # TODO(manevi): Bind this function.
    "mj_parseXMLString",
    "mj_recompile",  # TODO(manevi): Bind this function.
    "mj_saveXML",  # TODO(manevi): Bind this function.
    "mj_saveXMLString",  # TODO(manevi): Bind this function.
    # go/keep-sorted end
)

# Omitted because not very useful
_SKIPPED_WRITABLE_ERROR: tuple[str, ...] = (
    "mj_printSchema",
)

# Omitted thread management functions
_SKIPPED_THREAD_FUNCTIONS: tuple[str, ...] = (
    # go/keep-sorted start
    "mju_bindThreadPool",
    "mju_defaultTask",
    "mju_taskJoin",
    "mju_threadPoolCreate",
    "mju_threadPoolDestroy",
    "mju_threadPoolEnqueue",
    # go/keep-sorted end
)

# Omitted asset cache functions
_SKIPPED_ASSET_CACHE_FUNCTIONS: tuple[str, ...] = (
    # go/keep-sorted start
    "mj_clearCache",
    "mj_getCache",
    "mj_getCacheCapacity",
    "mj_getCacheSize",
    "mj_setCacheCapacity",
    # go/keep-sorted end
)

# Omitted Virtual Filesystem (VFS) functions
_SKIPPED_VFS_FUNCTIONS: tuple[str, ...] = (
    # go/keep-sorted start
    "mj_addBufferVFS",
    "mj_addFileVFS",
    "mj_defaultVFS",
    "mj_deleteFileVFS",
    "mj_deleteVFS",
    # go/keep-sorted end
)

# Omitted irrelevant visual functions
_SKIPPED_VISUAL_FUNCTIONS: tuple[str, ...] = (
    # go/keep-sorted start
    "mjv_averageCamera",
    "mjv_copyData",
    "mjv_copyModel",
    "mjv_defaultScene",
    "mjv_freeScene",
    "mjv_makeScene",
    # go/keep-sorted end
)

_SKIPPED_MEMORY_FUNCTIONS: tuple[str, ...] = (
    # go/keep-sorted start
    "mj_freeLastXML",
    "mj_freeStack",
    "mj_loadModel",
    "mj_loadModelBuffer",
    "mj_markStack",
    "mj_saveModel",
    "mj_stackAllocByte",
    "mj_stackAllocInt",
    "mj_stackAllocNum",
    "mj_warning",
    "mjs_bodyToFrame",
    "mju_boxQPmalloc",
    "mju_clearHandlers",
    "mju_error",
    "mju_error_i",
    "mju_error_s",
    "mju_free",
    "mju_malloc",
    "mju_strncpy",
    "mju_warning",
    "mju_warning_i",
    "mju_warning_s",
    # go/keep-sorted end
)

_SKIPPED_GETTERS_AND_SETTERS: tuple[str, ...] = (
    # go/keep-sorted start
    "mjs_appendFloatVec",
    "mjs_appendIntVec",
    "mjs_appendString",
    "mjs_getDouble",
    "mjs_getPluginAttributes",
    "mjs_getString",
    "mjs_getUserValue",
    "mjs_setBuffer",
    "mjs_setDouble",
    "mjs_setFloat",
    "mjs_setInStringVec",
    "mjs_setInt",
    "mjs_setPluginAttributes",
    "mjs_setString",
    "mjs_setStringVec",
    "mjs_setUserValue",
    # go/keep-sorted end
)

_SKIPPED_UTILITY_FUNCTIONS: tuple[str, ...] = (
    # go/keep-sorted start
    "mju_getXMLDependencies",
    # go/keep-sorted end
)

# Functions that require special wrappers.
# These functions are not bound automatically but are written by hand instead.
MANUAL_WRAPPER_FUNCTIONS: tuple[str, ...] = (
    # go/keep-sorted start
    "mj_saveLastXML",
    "mj_setLengthRange",
    "mju_error",
    # go/keep-sorted end
)

# List of functions that should be skipped during the code generation process.
SKIPPED_FUNCTIONS: tuple[str, ...] = (
    _SKIPPED_CLASS_METHODS
    + _SKIPPED_THREAD_FUNCTIONS
    + _SKIPPED_MEMORY_FUNCTIONS
    + _SKIPPED_PLUGIN_FUNCTIONS
    + _SKIPPED_GETTERS_AND_SETTERS
    + _SKIPPED_VISUAL_FUNCTIONS
    + _SKIPPED_ASSET_CACHE_FUNCTIONS
    + _SKIPPED_VFS_FUNCTIONS
    + _SKIPPED_WRITABLE_ERROR
    + _SKIPPED_UTILITY_FUNCTIONS
)

# List of structs that should be skipped during the code generation process.
SKIPPED_STRUCTS: tuple[str, ...] = (
    # go/keep-sorted start
    "mjCache",
    "mjSDF",
    "mjTask",
    "mjThreadPool",
    "mjUI",
    "mjVFS",
    "mjrContext",
    "mjrRect",
    "mjuiDef",
    "mjuiItem",
    "mjuiSection",
    "mjuiState",
    "mjuiThemeColor",
    "mjuiThemeSpacing"
    # go/keep-sorted end
)

# Structs for which header generation is done manually.
# mjvScene is included here because buffer sizes need to be calculated based on
# introspect doc strings, which was considered a brittle unreliable solution in
# the past.
MANUAL_STRUCTS_HEADERS: tuple[str, ...] = (
    "mjvScene",
)
# Structs for which source code generation is done manually.
MANUAL_STRUCTS_SOURCES: tuple[str, ...] = (
    "mjData",
    "mjModel",
    "mjvScene",
    "mjSpec",
)

# Dictionary that maps anonymous structs to their parent struct and field name.
# Anonymous structs are not defined as independent structs in the MuJoCo
# codebase, but they are part of other structs. This dictionary is used to
# handle them as if they were independent structs.
ANONYMOUS_STRUCTS: Dict[str, Dict[str, str]] = {
    # go/keep-sorted start
    "mjVisualGlobal": {"parent": "mjVisual", "field_name": "global"},
    "mjVisualHeadlight": {"parent": "mjVisual", "field_name": "headlight"},
    "mjVisualMap": {"parent": "mjVisual", "field_name": "map"},
    "mjVisualQuality": {"parent": "mjVisual", "field_name": "quality"},
    "mjVisualRgba": {"parent": "mjVisual", "field_name": "rgba"},
    "mjVisualScale": {"parent": "mjVisual", "field_name": "scale"},
    # go/keep-sorted end
}

# This list is created by subtracting the skipped structs from the list of all
# structs and adding the anonymous structs.
STRUCTS_TO_BIND: list[str] = list(
    set(introspect_structs.STRUCTS.keys())
    .union(ANONYMOUS_STRUCTS.keys())
    .difference(set(SKIPPED_STRUCTS))
)

# List of structs that do not have a default constructor.
NO_DEFAULT_CONSTRUCTORS: tuple[str, ...] = (
    # go/keep-sorted start
    "mjContact",
    "mjSolverStat",
    "mjStatistic",
    "mjTimerStat",
    "mjWarningStat",
    "mjsCompiler",
    "mjsDefault",
    "mjsElement",
    "mjsExclude",
    "mjsWrap",
    "mjvGLCamera",
    "mjvLight",
    # go/keep-sorted end
)

# List of `mjData` fields where the array size should be obtained from other
# `mjData` members, instead of from `mjModel` members. This is typically the
# case for fields that are dynamically allocated during the simulation.
MJDATA_SIZES: tuple[str, ...] = (
    # go/keep-sorted start
    "contact",
    "efc_AR",
    "efc_AR_colind",
    "efc_AR_rowadr",
    "efc_AR_rownnz",
    "efc_D",
    "efc_J",
    "efc_JT",
    "efc_JT_colind",
    "efc_J_colind",
    "efc_J_rowadr",
    "efc_J_rownnz",
    "efc_J_rowsuper",
    "efc_KBIP",
    "efc_R",
    "efc_aref",
    "efc_b",
    "efc_diagApprox",
    "efc_force",
    "efc_frictionloss",
    "efc_id",
    "efc_island",
    "efc_margin",
    "efc_pos",
    "efc_state",
    "efc_type",
    "efc_vel",
    "iLDiagInv",
    "iM_rowadr",
    "iM_rownnz",
    "iacc",
    "iacc_smooth",
    "iefc_D",
    "iefc_J",
    "iefc_JT",
    "iefc_JT_colind",
    "iefc_JT_rowadr",
    "iefc_JT_rownnz",
    "iefc_JT_rowsuper",
    "iefc_J_colind",
    "iefc_J_rowadr",
    "iefc_J_rownnz",
    "iefc_J_rowsuper",
    "iefc_R",
    "iefc_aref",
    "iefc_force",
    "iefc_frictionloss",
    "iefc_id",
    "iefc_state",
    "iefc_type",
    "ifrc_constraint",
    "ifrc_smooth",
    "island_dofadr",
    "island_dofnum",
    "island_efcadr",
    "island_efcind",
    "island_efcnum",
    "island_idofadr",
    "island_iefcadr",
    "island_itreeadr",
    "island_ne",
    "island_nefc",
    "island_nf",
    "island_ntree",
    "island_nv",
    "map_efc2iefc",
    "map_iefc2efc",
    # go/keep-sorted end
)

# Fields that should be entirely omitted from the bindings.
SKIPPED_FIELDS: Dict[str, list[str]] = {}

# Fields handled manually in template file struct declaration.
MANUAL_FIELDS: Dict[str, list[str]] = {
    # go/keep-sorted start
    "MjData": ["contact"],
    "MjvScene": [
        # go/keep-sorted start
        "camera",
        "flexedge",
        "flexedgeadr",
        "flexedgenum",
        "flexface",
        "flexfaceadr",
        "flexfacenum",
        "flexfaceused",
        "flexnormal",
        "flextexcoord",
        "flexvert",
        "flexvertadr",
        "flexvertnum",
        "geomorder",
        "geoms",
        "lights",
        "model",
        "skinfacenum",
        "skinnormal",
        "skinvert",
        "skinvertadr",
        "skinvertnum",
        # go/keep-sorted end
    ],
    # go/keep-sorted end
}

# Dictionary that maps byte array fields to their corresponding size members.
# When generating the code for these fields, a specific cast to `uint8_t*` is
# required for embind. This dictionary is used to register those fields and
# their sizes.
BYTE_FIELDS: Dict[str, Dict[str, str]] = {
    "buffer": {"size": "nbuffer"},
    "arena": {"size": "narena"},
}

# pyformat: disable
# Dictionary mapping function names to their boundcheck code.
FUNCTION_BOUNDS_CHECKS: Dict[str, str] = {
    "mj_solveM": """
  CHECK_SIZES(x, y);
  CHECK_DIVISIBLE(x, m.nv());
  int n = x_div.quot;
    """.strip(),
    "mj_solveM2": """
  CHECK_SIZES(x, y);
  CHECK_SIZE(sqrtInvD, m.nv());
  CHECK_DIVISIBLE(x, m.nv());
  int n = x_div.quot;
    """.strip(),
    "mju_add": """
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  int n = res_.size();
    """.strip(),
    "mju_addScl": """
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  int n = res_.size();
    """.strip(),
    "mju_addTo": """
  CHECK_SIZES(res, vec);
  int n = res_.size();
    """.strip(),
    "mju_addToScl": """
  CHECK_SIZES(res, vec);
  int n = res_.size();
    """.strip(),
    "mju_boxQP": """
  CHECK_SIZES(lower, res);
  CHECK_SIZES(upper, res);
  CHECK_SIZES(index, res);
  CHECK_SIZE(R, res_.size() * (res_.size() + 7))
  CHECK_PERFECT_SQUARE(H);
  CHECK_SIZES(g, res);
  int n = res_.size();
    """.strip(),
    "mju_cholFactor": """
  CHECK_PERFECT_SQUARE(mat);
  int n = mat_sqrt;
    """.strip(),
    "mju_cholSolve": """
  CHECK_PERFECT_SQUARE(mat);
  CHECK_SIZE(res, mat_sqrt);
  CHECK_SIZE(vec, mat_sqrt);
  int n = mat_sqrt;
    """.strip(),
    "mju_cholUpdate": """
  CHECK_PERFECT_SQUARE(mat);
  CHECK_SIZE(x, mat_sqrt);
  int n = mat_sqrt;
    """.strip(),
    "mju_copy": """
  CHECK_SIZES(res, vec);
  int n = res_.size();
    """.strip(),
    "mju_d2n": """
  CHECK_SIZES(res, vec);
  int n = res_.size();
    """.strip(),
    "mju_decodePyramid": """
  CHECK_SIZE(pyramid, 2 * mu_.size());
  CHECK_SIZE(force, mu_.size() + 1);
  int dim = mu_.size();
    """.strip(),
    "mju_dot": """
  CHECK_SIZES(vec1, vec2);
  int n = vec1_.size();
    """.strip(),
    "mju_encodePyramid": """
  CHECK_SIZE(pyramid, 2 * mu_.size());
  CHECK_SIZE(force, mu_.size() + 1);
  int dim = mu_.size();
    """.strip(),
    "mju_eye": """
  CHECK_PERFECT_SQUARE(mat);
  int n = mat_sqrt;
    """.strip(),
    "mju_f2n": """
  CHECK_SIZES(res, vec);
  int n = res_.size();
    """.strip(),
    "mju_mulVecMatVec": """
  CHECK_SIZES(vec1, vec2);
  CHECK_SIZE(mat, vec1_.size() * vec2_.size());
  int n = vec1_.size();
    """.strip(),
    "mju_n2d": """
  CHECK_SIZES(res, vec);
  int n = res_.size();
    """.strip(),
    "mju_n2f": """
  CHECK_SIZES(res, vec);
  int n = res_.size();
    """.strip(),
    "mju_printMatSparse": """
  CHECK_SIZES(rownnz, rowadr);
  int nr = rowadr_.size();
    """.strip(),
    "mju_scl": """
  CHECK_SIZES(res, vec);
  int n = res_.size();
    """.strip(),
    "mju_sub": """
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  int n = res_.size();
    """.strip(),
    "mju_subFrom": """
  CHECK_SIZES(res, vec);
  int n = res_.size();
    """.strip(),
    "mju_insertionSort": "int n = list_.size();",
    "mju_insertionSortInt": "int n = list_.size();",
    "mju_fill": "int n = res_.size();",
    "mju_dense2sparse": """
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(rownnz, nr);
  CHECK_SIZE(rowadr, nr);
  CHECK_SIZE(colind, res_.size());
  int nnz = res_.size();
    """.strip(),
    "mj_addM": """
  CHECK_SIZE(rownnz, m.nv());
  CHECK_SIZE(rowadr, m.nv());
  CHECK_SIZE(colind, m.nM());
  CHECK_SIZE(dst, m.nM());
    """.strip(),
    "mj_angmomMat": """
  CHECK_SIZE(mat, m.nv() * 3);
    """.strip(),
    "mj_applyFT": """
  CHECK_SIZE(qfrc_target, m.nv());
  CHECK_SIZE(force, 3);
  CHECK_SIZE(torque, 3);
  CHECK_SIZE(point, 3);
    """.strip(),
    "mj_constraintUpdate": """
  CHECK_SIZE(cost, 1);
  CHECK_SIZE(jar, d.nefc());
    """.strip(),
    "mj_differentiatePos": """
  CHECK_SIZE(qvel, m.nv());
  CHECK_SIZE(qpos1, m.nq());
  CHECK_SIZE(qpos2, m.nq());
    """.strip(),
    "mj_fullM": """
  CHECK_SIZE(M, m.nM());
  CHECK_SIZE(dst, m.nv() * m.nv());
    """.strip(),
    "mj_geomDistance": """
  CHECK_SIZE(fromto, 6);
    """.strip(),
    "mj_getState": """
  CHECK_SIZE(state, mj_stateSize(m.get(), sig));
    """.strip(),
    "mj_integratePos": """
  CHECK_SIZE(qpos, m.nq());
  CHECK_SIZE(qvel, m.nv());
    """.strip(),
    "mj_jac": """
  CHECK_SIZE(point, 3);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
    """.strip(),
    "mj_jacBody": """
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
    """.strip(),
    "mj_jacBodyCom": """
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
    """.strip(),
    "mj_jacDot": """
  CHECK_SIZE(point, 3);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
    """.strip(),
    "mj_jacGeom": """
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
    """.strip(),
    "mj_jacPointAxis": """
  CHECK_SIZE(point, 3);
  CHECK_SIZE(axis, 3);
  CHECK_SIZE(jacPoint, m.nv() * 3);
  CHECK_SIZE(jacAxis, m.nv() * 3);
    """.strip(),
    "mj_jacSite": """
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
    """.strip(),
    "mj_jacSubtreeCom": """
  CHECK_SIZE(jacp, m.nv() * 3);
    """.strip(),
    "mj_mulJacTVec": """
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, d.nefc());
    """.strip(),
    "mj_mulJacVec": """
  CHECK_SIZE(res, d.nefc());
  CHECK_SIZE(vec, m.nv());
    """.strip(),
    "mj_mulM": """
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, m.nv());
    """.strip(),
    "mj_mulM2": """
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, m.nv());
    """.strip(),
    "mj_multiRay": """
  CHECK_SIZE(dist, nray);
  CHECK_SIZE(geomid, nray);
  CHECK_SIZE(vec, 3 * nray);
    """.strip(),
    "mj_normalizeQuat": """
  CHECK_SIZE(qpos, m.nq());
    """.strip(),
    "mj_rne": """
  CHECK_SIZE(result, m.nv());
    """.strip(),
    "mj_setState": """
  CHECK_SIZE(state, mj_stateSize(m.get(), sig));
    """.strip(),
    "mjd_inverseFD": """
  CHECK_SIZE(DfDq, m.nv() * m.nv());
  CHECK_SIZE(DfDv, m.nv() * m.nv());
  CHECK_SIZE(DfDa, m.nv() * m.nv());
  CHECK_SIZE(DsDq, m.nv() * m.nsensordata());
  CHECK_SIZE(DsDv, m.nv() * m.nsensordata());
  CHECK_SIZE(DsDa, m.nv() * m.nsensordata());
  CHECK_SIZE(DmDq, m.nv() * m.nM());
    """.strip(),
    "mjd_subQuat": """
  CHECK_SIZE(qa, 4);
  CHECK_SIZE(qb, 4);
  CHECK_SIZE(Da, 9);
  CHECK_SIZE(Db, 9);
    """.strip(),
    "mjd_transitionFD": """
  CHECK_SIZE(A, (2 * m.nv() + m.na()) * (2 * m.nv() + m.na()));
  CHECK_SIZE(B, (2 * m.nv() + m.na()) * m.nu());
  CHECK_SIZE(C, m.nsensordata() * (2 * m.nv() + m.na()));
  CHECK_SIZE(D, m.nsensordata() * m.nu());
    """.strip(),
    "mju_band2Dense": """
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal * ntotal);
    """.strip(),
    "mju_bandMulMatVec": """
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal * nvec);
  CHECK_SIZE(vec, ntotal * nvec);
    """.strip(),
    "mju_cholFactorBand": """
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
    """.strip(),
    "mju_cholSolveBand": """
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal);
  CHECK_SIZE(vec, ntotal);
    """.strip(),
    "mju_dense2Band": """
  CHECK_SIZE(mat, ntotal * ntotal);
  CHECK_SIZE(res, (ntotal - ndense) * nband + ndense * ntotal);
    """.strip(),
    "mju_mulMatMat": """
  CHECK_SIZE(res, r1 * c2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, c1 * c2);
    """.strip(),
    "mju_mulMatMatT": """
  CHECK_SIZE(res, r1 * r2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, r2 * c1);
    """.strip(),
    "mju_mulMatTMat": """
  CHECK_SIZE(res, c1 * c2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, r1 * c2);
    """.strip(),
    "mju_mulMatTVec": """
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nc);
  CHECK_SIZE(vec, nr);
    """.strip(),
    "mju_mulMatVec": """
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nr);
  CHECK_SIZE(vec, nc);
    """.strip(),
    "mju_sparse2dense": """
  CHECK_SIZE(res, nr * nc);
  CHECK_SIZE(rownnz, nr);
  CHECK_SIZE(rowadr, nr);
    """.strip(),
    "mju_sqrMatTD": """
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nc * nc);
  CHECK_SIZE(diag, nr);
    """.strip(),
    "mju_symmetrize": """
  CHECK_SIZE(mat, n * n);
  CHECK_SIZE(res, n * n);
    """.strip(),
    "mju_transpose": """
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nr * nc);
    """.strip(),
}
# pyformat: enable
