// Copyright 2024 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SORT_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SORT_H_

// threshold size of a run to do insertion sort on
#define _mjRUNSIZE 32

// insertion sort sub-macro that runs on a sub-array [start, ..., end)
#define _mjINSERTION_SORT(type, arr, start, end, cmp, context)                                     \
{                                                                                                  \
  for (int j = start + 1; j < end; j++) {                                                          \
    type tmp = arr[j];                                                                             \
    int k = j - 1;                                                                                 \
    for (; k >= start && cmp(arr + k, &tmp, context) > 0; k--) {                                   \
      arr[k + 1] = arr[k];                                                                         \
    }                                                                                              \
    arr[k + 1] = tmp;                                                                              \
  }                                                                                                \
}

// sub-macro that merges two sub-sorted arrays [start, ..., mid), [mid, ..., end) together
#define _mjMERGE(type, src, dest, start, mid, end, cmp, context)                                   \
{                                                                                                  \
  int i = start, j = mid, k = start;                                                               \
  while (i < mid && j < end) {                                                                     \
    if (cmp(src + i, src + j, context) <= 0) {                                                     \
       dest[k++] = src[i++];                                                                       \
    } else {                                                                                       \
      dest[k++] = src[j++];                                                                        \
    }                                                                                              \
  }                                                                                                \
  if      (i < mid) memcpy(dest + k, src + i, (mid - i) * sizeof(type));                           \
  else if (j < end) memcpy(dest + k, src + j, (end - j) * sizeof(type));                           \
}

// defines an inline stable sorting function via tiled merge sorting (timsort)
// function is of form:
// void name(type* arr, type* buf, int n, void* context)
// where arr is the array of size n to be sorted inplace and buf is a buffer of size n.
#define mjSORT(name, type, cmp)                                                                    \
  static inline void name(type* arr, type* buf, int n, void* context) {                            \
    for (int start = 0; start < n; start += _mjRUNSIZE) {                                          \
      int end = (start + _mjRUNSIZE < n) ? start + _mjRUNSIZE : n;                                 \
      _mjINSERTION_SORT(type, arr, start, end, cmp, context);                                      \
    }                                                                                              \
    type* src = arr, *dest = buf, *tmp;                                                            \
    for (int len = _mjRUNSIZE; len < n; len *= 2) {                                                \
      for (int start = 0; start < n; start += 2*len) {                                             \
        int mid = start + len;                                                                     \
        int end = (start + 2*len < n) ? start + 2*len : n;                                         \
        if (mid < end) {                                                                           \
          _mjMERGE(type, src, dest, start, mid, end, cmp, context);                                \
        } else {                                                                                   \
          memcpy(dest + start, src + start, (end - start) * sizeof(type));                         \
        }                                                                                          \
      }                                                                                            \
      tmp = src; src = dest; dest = tmp;                                                           \
    }                                                                                              \
    if (src != arr) memcpy(arr, src, n * sizeof(type));                                            \
  }

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SORT_H_
