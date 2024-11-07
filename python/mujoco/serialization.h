// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_PYTHON_SERIALIZATION_H_
#define MUJOCO_PYTHON_SERIALIZATION_H_

#include <cstddef>
#include <iostream>

#include <mujoco/mjtnum.h>

namespace mujoco::python::_impl {

// For now, the serialization format is architecture- and version-dependent:
// It assumes that the writer and the reader have the same endianess, same
// numeric value sizes and same struct definitions.
// It is not safe to load serialized data from another version of MuJoCo or from
// a machine with a different architecture.

static_assert(sizeof(char) == 1);
static_assert(sizeof(int) == 4);
static_assert(sizeof(mjtNum) == 8);

inline void WriteChar(std::ostream& output, char c) {
  output.write(&c, 1);
}

inline char ReadChar(std::istream& input) {
  char c = '\0';
  input.read(&c, 1);
  return c;
}

inline void WriteInt(std::ostream& output, std::size_t i) {
  output.write(reinterpret_cast<char*>(&i), sizeof(std::size_t));
}

inline std::size_t ReadInt(std::istream& input) {
  std::size_t i = 0;
  input.read(reinterpret_cast<char*>(&i), sizeof(std::size_t));
  return i;
}

inline void WriteBytes(std::ostream& output, const void* src,
                       std::size_t nbytes) {
  // Start by writing nbytes itself, so it can be validated at the time of
  // reading.
  WriteInt(output, nbytes);
  if (src) {
    output.write(reinterpret_cast<const char*>(src), nbytes);
  }
}

inline void ReadBytes(std::istream& input, void* dest, std::size_t nbytes) {
  std::size_t actual_nbytes = ReadInt(input);
  if (actual_nbytes != nbytes) {
    input.setstate(input.rdstate() | std::ios_base::failbit);
    return;
  }
  input.read(reinterpret_cast<char*>(dest), nbytes);
}

}  // namespace mujoco::python::_impl

#endif  // MUJOCO_PYTHON_SERIALIZATION_H_
