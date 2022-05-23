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

set(MUJOCO_XML_SRCS
    xml_api.cc
    xml_api.h
    xml_base.cc
    xml_base.h
    xml.cc
    xml.h
    xml_native_reader.cc
    xml_native_reader.h
    xml_numeric_format.cc
    xml_numeric_format.h
    xml_native_writer.cc
    xml_native_writer.h
    xml_urdf.cc
    xml_urdf.h
    xml_util.cc
    xml_util.h
)

target_sources(mujoco PRIVATE ${MUJOCO_XML_SRCS})
