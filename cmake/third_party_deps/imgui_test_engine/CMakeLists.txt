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

# The engine sources live under the imgui_test_engine/ subdirectory of the repo.
set(TE_DIR ${imgui_test_engine_SOURCE_DIR}/imgui_test_engine)

add_library(imgui_test_engine STATIC)
target_include_directories(imgui_test_engine
  PUBLIC
    ${imgui_test_engine_SOURCE_DIR}
)
target_sources(imgui_test_engine
  PRIVATE
    ${TE_DIR}/imgui_te_context.cpp
    ${TE_DIR}/imgui_te_coroutine.cpp
    ${TE_DIR}/imgui_te_engine.cpp
    ${TE_DIR}/imgui_te_exporters.cpp
    ${TE_DIR}/imgui_te_perftool.cpp
    ${TE_DIR}/imgui_te_ui.cpp
    ${TE_DIR}/imgui_te_utils.cpp
    ${TE_DIR}/imgui_capture_tool.cpp
)
# Enable the built-in std::thread coroutine backend so ImGuiTestEngine_Start can
# register CoroutineFuncs (otherwise it asserts at startup). PUBLIC so the macro
# is consistent everywhere the engine headers are seen.
target_compile_definitions(imgui_test_engine
  PUBLIC
    IMGUI_TEST_ENGINE_ENABLE_COROUTINE_STDTHREAD_IMPL=1
)
# dear_imgui already carries IMGUI_ENABLE_TEST_ENGINE as a PUBLIC definition, so
# linking it both pulls in the imgui headers/include dirs and the macro.
target_link_libraries(imgui_test_engine
  PUBLIC
    dear_imgui
)
