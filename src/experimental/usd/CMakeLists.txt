# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# --- Global Configuration ---
# Plugin target names (used for library and plugInfo.json)
set(MJCF_PLUGIN_TARGET_NAME usdMjcf)
set(MJC_PHYSICS_PLUGIN_TARGET_NAME mjcPhysics)

# Installation directory for USD plugins
set(MJ_USD_INSTALL_DIR_LIB ${CMAKE_INSTALL_LIBDIR}/mujoco-usd-resources)

# Common OpenUSD libraries
set(OPENUSD_CORE_LIBS
  usd ar kind tf gf vt usdShade usdLux usdGeom usdPhysics
)

# We need to set the visibility to default until core type symbol visibility
# is resolved in OpenUSD https://github.com/PixarAnimationStudios/OpenUSD/issues/1475
set(DEFAULT_CXX_VISIBILITY_PROPS
  CXX_VISIBILITY_PRESET default
)

# --- Helper Functions ---
function(configure_and_install_usd_plugin_info plugin_name source_subpath install_base_dir)
  # Determine shared library prefix
  if(CMAKE_SHARED_LIBRARY_PREFIX)
    set(LIB_PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX})
  else()
    set(LIB_PREFIX "")
  endif()

  set(PLUG_INFO_LIBRARY_PATH "../../${LIB_PREFIX}${plugin_name}${CMAKE_SHARED_LIBRARY_SUFFIX}")
  set(OUTPUT_DIR "${CMAKE_BINARY_DIR}/${install_base_dir}/${plugin_name}")
  set(OUTPUT_FILE "${OUTPUT_DIR}/plugInfo.json")

  # Create the output directory if it doesn't exist
  file(MAKE_DIRECTORY "${OUTPUT_DIR}")

  # Configure the plugInfo.json file to replace @PLUG_INFO_LIBRARY_PATH@ instances.
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/${source_subpath}/plugInfo.json"
    "${OUTPUT_FILE}"
    @ONLY
  )

  install(FILES "${OUTPUT_FILE}"
    DESTINATION "${install_base_dir}/${plugin_name}"
  )
endfunction()

# Function to apply Houdini-specific link directories and include paths
function(configure_houdini_target target_name link_scope include_scope)
  set(HOUDINI_LIBS_DIR "${HOUDINI_HFS_DIR}/../Libraries")
  get_filename_component(HOUDINI_LIBS_DIR "${HOUDINI_LIBS_DIR}" ABSOLUTE) # Normalize the path

  # Assume everyone using Houdini on 3.11 for now.
  set(USD_MJCF_PYTHON_LIB python3.11)
  set(USD_MJCF_PYTHON_LIB_NUMBER python311) # For boost library names
  set(PYTHON_LIB "${HOUDINI_HFS_DIR}/Frameworks/Python.framework/Versions/3.11/Python")
  set(PXR_LIB_PREFIX "pxr_")

  # Houdini-specific PXR and other libraries
  set(HOUDINI_SPECIFIC_LIBS
    ${PXR_LIB_PREFIX}usd
    ${PXR_LIB_PREFIX}ar
    ${PXR_LIB_PREFIX}kind
    ${PXR_LIB_PREFIX}tf
    ${PXR_LIB_PREFIX}gf
    ${PXR_LIB_PREFIX}vt
    ${PXR_LIB_PREFIX}sdf
    ${PXR_LIB_PREFIX}usdShade
    ${PXR_LIB_PREFIX}usdLux
    ${PXR_LIB_PREFIX}usdGeom
    ${PXR_LIB_PREFIX}usdImaging
    ${PXR_LIB_PREFIX}usdPhysics
    tbb
    hboost_${USD_MJCF_PYTHON_LIB_NUMBER}
    ${PYTHON_LIB}
  )

  target_link_directories(${target_name} ${link_scope} ${HOUDINI_LIBS_DIR})
  target_include_directories(${target_name} ${include_scope}
    "${HOUDINI_HFS_DIR}/toolkit/include"
    "${HOUDINI_HFS_DIR}/toolkit/include/python3.11"
  )
  target_link_libraries(${target_name} ${link_scope}
    ${HOUDINI_SPECIFIC_LIBS}
    mujoco
    tinyxml2
  )
endfunction()

## usdMjcf

add_library(${MJCF_PLUGIN_TARGET_NAME} SHARED)
target_sources(${MJCF_PLUGIN_TARGET_NAME} PRIVATE
  plugins/mjcf/mjcf_file_format.cc
  plugins/mjcf/mjcf_file_format.h
  plugins/mjcf/mujoco_to_usd.cc
  plugins/mjcf/mujoco_to_usd.h
  plugins/mjcf/utils.cc
  plugins/mjcf/utils.h
)
set_target_properties(${MJCF_PLUGIN_TARGET_NAME} PROPERTIES
  OUTPUT_NAME ${MJCF_PLUGIN_TARGET_NAME}
  ${DEFAULT_CXX_VISIBILITY_PROPS}
)
target_include_directories(${MJCF_PLUGIN_TARGET_NAME} PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/plugins"
)

## ----- mjcPhysics -----

add_library(${MJC_PHYSICS_PLUGIN_TARGET_NAME} SHARED)
target_sources(${MJC_PHYSICS_PLUGIN_TARGET_NAME} PRIVATE
  mjcPhysics/actuator.cpp
  mjcPhysics/collisionAPI.cpp
  mjcPhysics/imageableAPI.cpp
  mjcPhysics/jointAPI.cpp
  mjcPhysics/keyframe.cpp
  mjcPhysics/materialAPI.cpp
  mjcPhysics/meshCollisionAPI.cpp
  mjcPhysics/sceneAPI.cpp
  mjcPhysics/siteAPI.cpp
  mjcPhysics/tendon.cpp
  mjcPhysics/tokens.cpp
)
set_target_properties(${MJC_PHYSICS_PLUGIN_TARGET_NAME} PROPERTIES
  OUTPUT_NAME ${MJC_PHYSICS_PLUGIN_TARGET_NAME}
  ${DEFAULT_CXX_VISIBILITY_PROPS}
)
target_include_directories(${MJC_PHYSICS_PLUGIN_TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
)

## ----- mujoco USD utils -----

target_sources(mujoco PRIVATE
  kinematic_tree.cc
  kinematic_tree.h
  layer_sink.cc
  material_parsing.cc
  material_parsing.h
  usd_to_mjspec.cc
  utils.cc
  writer.cc
)

# pxr/usd/tf/hashset.h might include deprecated headers so disable this check
# when compiling with USD enabled.
target_compile_options(
     mujoco
     PRIVATE -Wno-deprecated
)

# Configure RPATH for macOS and UNIX.
if(APPLE)
  set_target_properties($(MJCF_PLUGIN_TARGET_NAME) PROPERTIES
    BUILD_RPATH "${USD_INSTALL_ROOT}"
    INSTALL_RPATH "@loader_path")

  set_target_properties($(MJC_PHYSICS_PLUGIN_TARGET_NAME) PROPERTIES
    BUILD_RPATH "${USD_INSTALL_ROOT}"
    INSTALL_RPATH "@loader_path")
elseif(UNIX)
  set_target_properties($(MJCF_PLUGIN_TARGET_NAME) PROPERTIES
    BUILD_RPATH "${USD_INSTALL_ROOT}"
    INSTALL_RPATH "$ORIGIN")

  set_target_properties($(MJC_PHYSICS_PLUGIN_TARGET_NAME) PROPERTIES
    BUILD_RPATH "${USD_INSTALL_ROOT}"
    INSTALL_RPATH "$ORIGIN")
endif()

target_link_libraries(${MJC_PHYSICS_PLUGIN_TARGET_NAME} PRIVATE
  ${OPENUSD_CORE_LIBS}
)

target_link_libraries(${MJCF_PLUGIN_TARGET_NAME} PRIVATE
  ${OPENUSD_CORE_LIBS}
  mujoco
  tinyxml2
  ${MJC_PHYSICS_PLUGIN_TARGET_NAME}
)

target_link_libraries(mujoco PUBLIC
  ${OPENUSD_CORE_LIBS}
  ${MJC_PHYSICS_PLUGIN_TARGET_NAME}
)

## Installation

# Generate and install plugInfo.json for each plugin
configure_and_install_usd_plugin_info(
  ${MJCF_PLUGIN_TARGET_NAME}
  plugins/mjcf
  ${MJ_USD_INSTALL_DIR_LIB}
)
configure_and_install_usd_plugin_info(
  ${MJC_PHYSICS_PLUGIN_TARGET_NAME}
  mjcPhysics
  ${MJ_USD_INSTALL_DIR_LIB}
)

# Install specific resources that are not plugInfo.json (like generatedSchema.usda)
install(FILES
  mjcPhysics/generatedSchema.usda
  DESTINATION ${MJ_USD_INSTALL_DIR_LIB}/mjcPhysics
)

# Install shared libraries
install(TARGETS
  ${MJCF_PLUGIN_TARGET_NAME}
  ${MJC_PHYSICS_PLUGIN_TARGET_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
