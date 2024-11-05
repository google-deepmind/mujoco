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
#define _mjMERGE(type, arr, buf, start, mid, end, cmp, context)                                    \
{                                                                                                  \
  int len1 = mid - start, len2 = end - mid;                                                        \
  type* left = buf, *right = buf + len1;                                                           \
  for (int i = 0; i < len1; i++) left[i] = arr[start + i];                                         \
  for (int i = 0; i < len2; i++) right[i] = arr[mid + i];                                          \
  int i = 0, j = 0, k = start;                                                                     \
  while (i < len1 && j < len2) {                                                                   \
    if (cmp(left + i, right + j, context) <= 0) {                                                  \
       arr[k++] = left[i++];                                                                       \
    } else {                                                                                       \
      arr[k++] = right[j++];                                                                       \
    }                                                                                              \
  }                                                                                                \
  while (i < len1) arr[k++] = left[i++];                                                           \
  while (j < len2) arr[k++] = right[j++];                                                          \
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
    for (int len = _mjRUNSIZE; len < n; len *= 2) {                                                \
      for (int start = 0; start < n; start += 2*len) {                                             \
        int mid = start + len;                                                                     \
        int end = (start + 2*len < n) ? start + 2*len : n;                                         \
        if (mid < end) {                                                                           \
          _mjMERGE(type, arr, buf, start, mid, end, cmp, context);                                 \
        }                                                                                          \
      }                                                                                            \
    }                                                                                              \
  }

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SORT_H_
