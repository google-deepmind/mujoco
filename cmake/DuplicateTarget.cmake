# Copyright 2022 DeepMind Technologies Limited
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
#
#.rst:
# duplicate_target
# ----------------------
#
# Duplicates the specified target.
#
#   duplicate_target(TARGET target NEW_TARGET_NAME new_target_name)
#
# Duplicates the specified target by creating a new target with the name
# specified by ``NEW_TARGET_NAME`` and copying all properties defined by
# ``TARGET``.
# The command has the following parameters:
#
# Arguments:
#  - ``TARGET`` Target to be duplicated.
#  - ``NEW_TARGET_NAME`` name of the new target.
#

if(COMMAND duplicate_target)
  return()
endif()

macro(duplicate_target)
  # Parse arguments.
  set(options)
  set(one_value_args TARGET NEW_TARGET_NAME)
  set(multi_value_args)
  cmake_parse_arguments(
    _ARGS
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  # Check required variables are defined.
  if(NOT _ARGS_TARGET)
    message(FATAL_ERROR "duplicate_target: TARGET must be specified.")
  endif()
  if(NOT _ARGS_NEW_TARGET_NAME)
    message(FATAL_ERROR "duplicate_target: NEW_TARGET_NAME must be specified.")
  endif()

  # Check for the target to exist.
  if(NOT TARGET ${_ARGS_TARGET})
    message(FATAL_ERROR "duplicate_target: TARGET ${_ARGS_TARGET} not found.")
  endif()

  # Non-exausitve list.
  set(PROPERTIES_TO_SKIP)
  get_target_property(TARGET_TYPE ${_ARGS_TARGET} TYPE)
  if(${TARGET_TYPE} STREQUAL "STATIC_LIBRARY")
    add_library(${_ARGS_NEW_TARGET_NAME} STATIC)
    list(APPEND PROPERTIES_TO_SKIP "IMPORTED_GLOBAL")
  elseif(${TARGET_TYPE} STREQUAL "SHARED_LIBRARY")
    add_library(${_ARGS_NEW_TARGET_NAME} SHARED)
    list(APPEND PROPERTIES_TO_SKIP "IMPORTED_GLOBAL")
  elseif(${TARGET_TYPE} STREQUAL "OBJECT_LIBRARY")
    add_library(${_ARGS_NEW_TARGET_NAME} OBJECT)
    list(APPEND PROPERTIES_TO_SKIP "IMPORTED_GLOBAL")
  elseif(${TARGET_TYPE} STREQUAL "INTERFACE_LIBRARY")
    add_library(${_ARGS_NEW_TARGET_NAME} INTERFACE)
  elseif(${TARGET_TYPE} STREQUAL "EXECUTABLE")
    add_executable(${_ARGS_NEW_TARGET_NAME})
    list(APPEND PROPERTIES_TO_SKIP "IMPORTED_GLOBAL")
  endif()

  set(IGNORED_PROPERTIES "HEADER_SETS;INTERFACE_HEADER_SETS;NAME;TYPE")

  # Get all CMake target properties.
  if(NOT CMAKE_ALL_PROPERTY_LIST)
    execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE ALL_PROPERTIES)

    # Convert command output into a CMake list
    string(
      REGEX
      REPLACE ";"
              "\\\\;"
              ALL_PROPERTIES
              "${ALL_PROPERTIES}"
    )
    string(
      REGEX
      REPLACE "\n"
              ";"
              ALL_PROPERTIES
              "${ALL_PROPERTIES}"
    )

    # Post process the properties. We:
    # - Remove all properties listed in ``IGNORED_PROPERTIES``.
    # - Remove LOCATION as it should not be accessed, See https://stackoverflow.com/questions/32197663/how-can-i-remove-the-the-location-property-may-not-be-read-from-target-error-i
    # - Substitute <CONFIG> with all the possible build types. This is needed for multi-config generators as they can change the type of build without invoking again CMake.
    set(CMAKE_ALL_PROPERTY_LIST "")
    foreach(PROPERTY ${ALL_PROPERTIES})
      # Skip reading the LOCAION property as they should not be read.
      # See https://stackoverflow.com/questions/32197663/how-can-i-remove-the-the-location-property-may-not-be-read-from-target-error-i
      if(PROPERTY STREQUAL "LOCATION"
         OR PROPERTY MATCHES "^LOCATION_"
         OR PROPERTY MATCHES "_LOCATION$"
      )
        continue()
      endif()

      set(_ignore OFF)
      foreach(_pro ${IGNORED_PROPERTIES})
        if(PROPERTY STREQUAL ${_pro})
          set(_ignore ON)
          break()
        endif()
      endforeach()

      if(_ignore)
        continue()
      endif()

      string(FIND ${PROPERTY} "<CONFIG>" FOUND)
      if(${FOUND} EQUAL -1)
        # Simply append the property.
        list(APPEND CMAKE_ALL_PROPERTY_LIST ${PROPERTY})
      else()
        # Iterate on the build types to add the correct property.
        foreach(BUILD_TYPE ${BUILD_TYPES})
          string(
            REPLACE "<CONFIG>"
                    "${BUILD_TYPE}"
                    CONFIG_PROPERTY
                    ${PROPERTY}
          )
          list(APPEND CMAKE_ALL_PROPERTY_LIST ${CONFIG_PROPERTY})
        endforeach()
      endif()

    endforeach()

  endif()

  foreach(PROPERTY ${CMAKE_ALL_PROPERTY_LIST})
    # Search if we should skip this property.

    list(
      FIND
      PROPERTIES_TO_SKIP
      ${PROPERTY}
      SHOULD_SKIP
    )
    if(${SHOULD_SKIP} GREATER_EQUAL 0)
      continue()
    endif()

    # First check if the property was set on the target
    get_property(
      PROPERTY_FOUND
      TARGET ${_ARGS_TARGET}
      PROPERTY ${PROPERTY}
      SET
    )
    if(PROPERTY_FOUND)
      get_target_property(PROPERTY_VALUE ${_ARGS_TARGET} ${PROPERTY})
      set_target_properties(${_ARGS_NEW_TARGET_NAME} PROPERTIES ${PROPERTY} "${PROPERTY_VALUE}")
    endif()
  endforeach()
endmacro()
