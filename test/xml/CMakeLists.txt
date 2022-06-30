# Copyright 2021 DeepMind Technologies Limited
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

mujoco_test(xml_api_test)
target_link_libraries(xml_api_test fixture gmock)

mujoco_test(xml_native_reader_test)
target_link_libraries(xml_native_reader_test fixture gmock)

mujoco_test(xml_native_writer_test)
target_link_libraries(
  xml_native_writer_test
  fixture
  gmock
  absl::flat_hash_set
)
