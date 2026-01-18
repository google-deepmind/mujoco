// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MUJOCO_INCLUDE_MJTNUM_H_
#define MUJOCO_INCLUDE_MJTNUM_H_

#include <stdint.h>


//---------------------------------- floating-point definition -------------------------------------

// floating point data type and minval
#ifndef mjUSESINGLE
  typedef double mjtNum;
  #define mjMINVAL    1E-15       // minimum value in any denominator
#else
  typedef float mjtNum;
  #define mjMINVAL    1E-15f
#endif



//---------------------------------- byte definition -----------------------------------------------

typedef unsigned char mjtByte;    // used for true/false



//---------------------------------- size definition -----------------------------------------------

typedef int64_t mjtSize;          // used for buffer sizes



#endif  // MUJOCO_INCLUDE_MJTNUM_H_
