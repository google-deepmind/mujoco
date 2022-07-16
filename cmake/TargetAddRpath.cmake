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
#[=======================================================================[.rst:
TargetAddRpath
----------------------

Add support to RPATH for the specified targets

.. command:: target_add_rpath

  Add support to RPATH for the specified targets::

.. code-block:: cmake
    target_add_rpath(target1 target2 ...
                     INSTALL_DIRECTORY install_directory
                     LIB_DIRS dir1 dir2 ...
                     [INSTALL_NAME_DIR [dir]]
                     [DEPENDS condition [condition]]
                     [USE_LINK_PATH])

  This function setups the RPATH paths for the specified targets. The use of
  RPATH allows to avoid using the (dangerous) environment variable
  ``LD_LIBRARY_PATH`` (or equivalent on macOS) when installing  without absolute
  paths.
  By using RPATH the installation can be relocated as the linker will look for
  (some of) the dependencies at run time.

  The function has the following parameters:

  Options:
   - ``USE_LINK_PATH``: if defined, the function will set
     ``INSTALL_RPATH_USE_LINK_PATH`` on all the specified targets, i.e. CMake
      will automatically adds to the RPATH the path to all the dependent
      libraries defined outside this project.

  Arguments:
   - ``INSTALL_DIRECTORY`` The directory where the specified targets will be
     installed.
   - ``LIB_DIRS`` list of directories to be added a search path to the RPATH.
     Note that the relative path between ``INSTALL_DIRECTORY`` and these
     directories will be added to the RPATH.
   - ``INSTALL_NAME_DIR`` directory where the libraries will be installed.
     This variable will be used only if ``CMAKE_SKIP_RPATH`` or
     ``CMAKE_SKIP_INSTALL_RPATH`` is set to ``TRUE`` as it will set the
     ``INSTALL_NAME_DIR`` on all targets.
   - ``DEPENDS`` list of conditions that should be ``TRUE`` to enable
     RPATH, for example ``FOO; NOT BAR``.

  Note: see https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
  and  https://gitlab.kitware.com/cmake/cmake/issues/16589 for further details.

#]=======================================================================]

if(COMMAND target_add_rpath)
  return()
endif()

function(_get_system_dirs _output_var)
  set(${_output_var}
      ${CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES}
      PARENT_SCOPE
  )
endfunction()

function(
  _get_rpath_relative_path
  _output_var
  _bin_dir
  _lib_dir
)
  file(
    RELATIVE_PATH
    _rel_path
    ${_bin_dir}
    ${_lib_dir}
  )
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(${_output_var}
        "@loader_path/${_rel_path}"
        PARENT_SCOPE
    )
  else()
    set(${_output_var}
        "\$ORIGIN/${_rel_path}"
        PARENT_SCOPE
    )
  endif()
endfunction()

function(target_add_rpath)
  set(_options USE_LINK_PATH)
  set(_oneValueArgs INSTALL_NAME_DIR INSTALL_DIRECTORY)
  set(_multiValueArgs TARGETS LIB_DIRS DEPENDS)

  cmake_parse_arguments(
    _ARGS
    "${_options}"
    "${_oneValueArgs}"
    "${_multiValueArgs}"
    "${ARGN}"
  )

  # Handle Apple-specific installation directory. Note that this disable proper RPATH.
  if(CMAKE_SKIP_RPATH OR CMAKE_SKIP_INSTALL_RPATH)
    if(DEFINED _ARGS_INSTALL_NAME_DIR)
      set_target_properties(${_ARGS_TARGETS} PROPERTIES INSTALL_NAME_DIR ${_ARGS_INSTALL_NAME_DIR})
    endif()
  endif()

  # If RPATH is disabled, do nothing and return.
  if(CMAKE_SKIP_RPATH OR (CMAKE_SKIP_INSTALL_RPATH AND CMAKE_SKIP_BUILD_RPATH))
    return()
  endif()

  # Check if the user requested RPATH for the specified targets.
  set(_enable_rpath ON)
  if(DEFINED _ARGS_DEPENDS)
    foreach(_cond ${_ARGS_DEPENDS})
      string(
        REGEX
        REPLACE " +"
                ";"
                _cond
                "${_cond}"
      )
      if(NOT (${_cond}))
        set(_enable_rpath OFF)
      endif()
    endforeach()
  endif()

  if(NOT _enable_rpath)
    return()
  endif()

  # Now enable RPATH for the specified targets.
  _get_system_dirs(_system_dirs)

  # We do this per target to preserve the original rpath setting.
  foreach(_target ${_ARGS_TARGETS})
    get_target_property(_install_rpath ${_target} INSTALL_RPATH)

    foreach(_lib_dir ${_ARGS_LIB_DIRS})
      # Check if the specified library path is a system directory. These are always searched so we do
      # not need to include them in the RPATH.
      list(
        FIND
        _system_dirs
        "${_lib_dir}"
        is_system_dir
      )
      if("${is_system_dir}" STREQUAL "-1")

        _get_rpath_relative_path(_bin_lib_rel_path ${_ARGS_INSTALL_DIRECTORY} ${_lib_dir})
        list(APPEND _install_rpath ${_bin_lib_rel_path})

      endif()
    endforeach()

    if(NOT
       "${_install_rpath}"
       STREQUAL
       ""
    )
      list(REMOVE_DUPLICATES _install_rpath)
    endif()

    set_target_properties(
      ${_target}
      PROPERTIES INSTALL_RPATH ${_install_rpath}
                 INSTALL_RPATH_USE_LINK_PATH ${_ARGS_USE_LINK_PATH}
                 BUILD_WITH_INSTALL_RPATH TRUE
                 MACOSX_RPATH ON # This is ON by default.
    )
  endforeach()

endfunction()
