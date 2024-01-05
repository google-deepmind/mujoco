// Copyright 2023 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_CONTAINER_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_CONTAINER_H_

#include <stddef.h>

#include <mujoco/mjdata.h>

#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------- mjArrayList ---------------------------------------------------
struct mjArrayList_ {
  // pointer to mjData to allow for growth of the list
  mjData* d;

  // size of element
  size_t element_size;

  // maximum number of elements
  size_t capacity;

  // number of elements in this list
  size_t size;

  // pointer to the next segment of the array list, NULL if last
  struct mjArrayList_* next_segment;

  // buffer for data in this segment stored in d->arena
  void* buffer;
};
typedef struct mjArrayList_ mjArrayList;

// stack allocate and initialize new mjArrayList
mjArrayList* mju_arrayListCreate(mjData* d, size_t element_size, size_t initial_capacity);

// returns total number of elements in mjArrayList
size_t mju_arrayListSize(const mjArrayList* array_list);

// copies an element into an mjArrayList
void mju_arrayListAdd(mjArrayList* array_list, void* element);

// returns a pointer to the element at the specified location of the arraylist
// NULL returned if index is not in the mjArrayList
void* mju_arrayListAt(const mjArrayList* array_list, size_t index);

#ifdef __cplusplus
}
#endif


#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_CONTAINER_H_
