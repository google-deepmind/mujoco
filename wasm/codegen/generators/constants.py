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

from typing import Dict, List, Set
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

_PLUGIN_FUNCTIONS: List[str] = [
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
]

# Functions that are bound as class methods
_CLASS_METHODS: List[str] = [
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
]

# Omitted because not very useful
_WRITABLE_ERROR: List[str] = [
    "mj_printSchema",
]

# Omitted thread management functions
_THREAD_FUNCTIONS: List[str] = [
    # go/keep-sorted start
    "mju_bindThreadPool",
    "mju_defaultTask",
    "mju_taskJoin",
    "mju_threadPoolCreate",
    "mju_threadPoolDestroy",
    "mju_threadPoolEnqueue",
    # go/keep-sorted end
]

# Omitted asset cache functions
_ASSET_CACHE_FUNCTIONS: List[str] = [
    # go/keep-sorted start
    "mj_clearCache",
    "mj_getCache",
    "mj_getCacheCapacity",
    "mj_getCacheSize",
    "mj_setCacheCapacity",
    # go/keep-sorted end
]

# Omitted Virtual Filesystem (VFS) functions
_VFS_FUNCTIONS: List[str] = [
    # go/keep-sorted start
    "mj_addBufferVFS",
    "mj_addFileVFS",
    "mj_defaultVFS",
    "mj_deleteFileVFS",
    "mj_deleteVFS",
    # go/keep-sorted end
]

# Omitted irrelevant visual functions
_VISUAL_FUNCTIONS: List[str] = [
    # go/keep-sorted start
    "mjv_averageCamera",
    "mjv_copyData",
    "mjv_copyModel",
    "mjv_defaultScene",
    "mjv_freeScene",
    "mjv_makeScene",
    # go/keep-sorted end
]

_MEMORY_FUNCTIONS: List[str] = [
    # go/keep-sorted start
    "mj_freeLastXML",
    "mj_freeStack",
    "mj_loadModel",
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
]

_GETTERS_AND_SETTERS: List[str] = [
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
]

_UTILITY_FUNCTIONS: List[str] = [
    # go/keep-sorted start
    "mju_getXMLDependencies",
    # go/keep-sorted end
]

# List of functions that should be skipped during the code generation process.
SKIPPED_FUNCTIONS: List[str] = (
    _CLASS_METHODS
    + _THREAD_FUNCTIONS
    + _MEMORY_FUNCTIONS
    + _PLUGIN_FUNCTIONS
    + _GETTERS_AND_SETTERS
    + _VISUAL_FUNCTIONS
    + _ASSET_CACHE_FUNCTIONS
    + _VFS_FUNCTIONS
    + _WRITABLE_ERROR
    + _UTILITY_FUNCTIONS
)

# Functions that require special wrappers to infer sizes and make additional
# validation checks. These functions are not bound automatically but are
# written by hand instead.
BOUNDCHECK_FUNCS: List[str] = [
    # go/keep-sorted start
    "mj_addM",
    "mj_angmomMat",
    "mj_applyFT",
    "mj_constraintUpdate",
    "mj_differentiatePos",
    "mj_fullM",
    "mj_geomDistance",
    "mj_getState",
    "mj_integratePos",
    "mj_jac",
    "mj_jacBody",
    "mj_jacBodyCom",
    "mj_jacDot",
    "mj_jacGeom",
    "mj_jacPointAxis",
    "mj_jacSite",
    "mj_jacSubtreeCom",
    "mj_mulJacTVec",
    "mj_mulJacVec",
    "mj_mulM",
    "mj_mulM2",
    "mj_multiRay",
    "mj_normalizeQuat",
    "mj_rne",
    "mj_saveLastXML",
    "mj_setLengthRange",
    "mj_setState",
    "mj_solveM",
    "mj_solveM2",
    "mjd_inverseFD",
    "mjd_subQuat",
    "mjd_transitionFD",
    "mju_L1",
    "mju_add",
    "mju_addScl",
    "mju_addTo",
    "mju_addToScl",
    "mju_band2Dense",
    "mju_bandMulMatVec",
    "mju_boxQP",
    "mju_cholFactor",
    "mju_cholFactorBand",
    "mju_cholSolve",
    "mju_cholSolveBand",
    "mju_cholUpdate",
    "mju_copy",
    "mju_d2n",
    "mju_decodePyramid",
    "mju_dense2Band",
    "mju_dense2sparse",
    "mju_dot",
    "mju_encodePyramid",
    "mju_eye",
    "mju_f2n",
    "mju_fill",
    "mju_insertionSort",
    "mju_insertionSortInt",
    "mju_isZero",
    "mju_mulMatMat",
    "mju_mulMatMatT",
    "mju_mulMatTMat",
    "mju_mulMatTVec",
    "mju_mulMatVec",
    "mju_mulVecMatVec",
    "mju_n2d",
    "mju_n2f",
    "mju_norm",
    "mju_normalize",
    "mju_printMatSparse",
    "mju_scl",
    "mju_sparse2dense",
    "mju_sqrMatTD",
    "mju_sub",
    "mju_subFrom",
    "mju_sum",
    "mju_symmetrize",
    "mju_transpose",
    "mju_zero",
    # go/keep-sorted end
]

# List of structs that should be skipped during the code generation process.
SKIPPED_STRUCTS: List[str] = [
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
]

# These structs require specific function calls for creation and/or deletion,
# or some of their fields need to be handled manually for now;
# making their wrapper constructors/destructors non-trivial.
MANUAL_STRUCTS: List[str] = [
    "MjData",
    "MjModel",
    "MjvScene",
    "MjSpec",
    "MjVisual",
]

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
STRUCTS_TO_BIND: List[str] = list(
    set(introspect_structs.STRUCTS.keys())
    .union(ANONYMOUS_STRUCTS.keys())
    .difference(set(SKIPPED_STRUCTS))
)

# List of structs that do not have a default constructor.
NO_DEFAULT_CONSTRUCTORS: List[str] = [
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
]

# List of `mjData` fields where the array size should be obtained from other
# `mjData` members, instead of from `mjModel` members. This is typically the
# case for fields that are dynamically allocated during the simulation.
MJDATA_SIZES: List[str] = [
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
]

# Fields that should be entirely omitted from the bindings.
SKIPPED_FIELDS: Dict[str, List[str]] = {}

# Fields handled manually in template file struct declaration.
MANUAL_FIELDS: Dict[str, List[str]] = {
    # go/keep-sorted start
    "MjData": ["solver", "timer", "warning", "contact"],
    "MjModel": ["opt", "vis", "stat"],
    "MjSpec": ["option", "visual", "stat", "element", "compiler"],
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
