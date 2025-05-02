# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# Plugin target name (used for library and plugInfo.json)
set(MJCF_PLUGIN_TARGET_NAME usdMjcf)

add_library(${MJCF_PLUGIN_TARGET_NAME} SHARED
  mjcf/mjcf_file_format.cc
  mjcf/mjcf_file_format.h
  mjcf/mujoco_to_usd.cc
  mjcf/mujoco_to_usd.h
  mjcf/utils.cc
  mjcf/utils.h
)

# We need to set the visibility to default until core type symbol visibility
# is resolved in OpenUSD https://github.com/PixarAnimationStudios/OpenUSD/issues/1475
# Otherwise we will run into issues during composition on MacOS due to std::type_info
# comparisons failing for pxr::TfTokenVector and the like that we place in SdfAbstractData.
set_target_properties(${MJCF_PLUGIN_TARGET_NAME} PROPERTIES
    OUTPUT_NAME ${MJCF_PLUGIN_TARGET_NAME}
    CXX_VISIBILITY_PRESET default
)

target_include_directories(${MJCF_PLUGIN_TARGET_NAME} PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

if (MUJOCO_USD_TARGET STREQUAL "USD")
  find_package(pxr REQUIRED)

  # --- Link Dependencies ---
  # Link against the necessary OpenUSD components
  target_link_libraries(${MJCF_PLUGIN_TARGET_NAME} PRIVATE
    usd
    ar
    kind
    tf
    gf
    vt
    usdShade
    usdLux
    usdGeom
    usdImaging
    usdPhysics
    mujoco
    tinyxml2
  )
elseif (MUJOCO_USD_TARGET STREQUAL "Houdini")

  if (NOT DEFINED ENV{HFS})
    message(FATAL_ERROR "Environment variable 'HFS' is not defined: $ENV{HFS}. Please run houdini_setup.")
  endif()

  # In Houdini, the Houdini package we would typically use via find_package
  # does not have all the USD dependencies that we need (namely UsdPhysics)
  # so we need to manually link all the required libraries.

  set(HFS_ENV "$ENV{HFS}")
  set(HOUDINI_LIBS "${HFS_ENV}/../Libraries")
  get_filename_component(HOUDINI_LIBS "${HOUDINI_LIBS}" ABSOLUTE) # Normalize the path
  target_link_directories(${MJCF_PLUGIN_TARGET_NAME} PRIVATE ${HOUDINI_LIBS})

  target_include_directories(${MJCF_PLUGIN_TARGET_NAME} PRIVATE
    "${HFS_ENV}/toolkit/include"
    "${HFS_ENV}/toolkit/include/python3.11"
  )

  # Assume everyone using Houdini on 3.11 for now.
  set(USD_MJCF_PYTHON_LIB python3.11)
  set(USD_MJCF_PYTHON_LIB_NUMBER python311)
  set(PYTHON_LIB "${HFS_ENV}/Frameworks/Python.framework/Versions/3.11/Python")
  set(PXR_LIB_PREFIX "pxr_")

  # --- Link Dependencies ---
  # Link against the necessary OpenUSD components
  target_link_libraries(${MJCF_PLUGIN_TARGET_NAME} PRIVATE
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
    mujoco
    tinyxml2
  )
endif()


# --- Generate plugInfo.json ---
if(CMAKE_SHARED_LIBRARY_PREFIX)
  set(LIB_PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX}) # Usually "lib" on Unix
else()
  set(LIB_PREFIX "")
endif()
set(PLUG_INFO_LIBRARY_PATH "${LIB_PREFIX}${MJCF_PLUGIN_TARGET_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")

# --- Installation ---

set(USD_PLUGIN_INSTALL_DIR_LIB ${CMAKE_INSTALL_LIBDIR}/usdMjcf)

message(STATUS "Copying plugInfo.json to ${CMAKE_BINARY_DIR}/${USD_PLUGIN_INSTALL_DIR_LIB}/plugInfo.json")
configure_file(
  mjcf/plugInfo.json
  ${CMAKE_BINARY_DIR}/${USD_PLUGIN_INSTALL_DIR_LIB}/plugInfo.json
)

install(FILES ${CMAKE_BINARY_DIR}/${USD_PLUGIN_INSTALL_DIR_LIB}/plugInfo.json DESTINATION ${USD_PLUGIN_INSTALL_DIR_LIB})

# Install shared lib and plugInfo to same location for simplicity.
install(TARGETS ${MJCF_PLUGIN_TARGET_NAME}
    LIBRARY DESTINATION ${USD_PLUGIN_INSTALL_DIR_LIB}
)

message(STATUS "USD MJCF Plugin will be installed to: ${CMAKE_INSTALL_PREFIX}/${USD_PLUGIN_INSTALL_DIR_LIB}")
message(STATUS "Make sure PXR_PLUGINPATH_NAME includes: ${CMAKE_INSTALL_PREFIX}/${USD_PLUGIN_INSTALL_DIR_LIB}")
