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

#include "src/engine/engine_sort.h"

#include <algorithm>
#include <array>
#include <vector>
#include <utility>

#include <gtest/gtest.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using std::array;
using std::vector;
using EngineSortTest = MujocoTest;

constexpr int kPermutationSize = 5;  // 5! = 120 checks

constexpr int factorial() {
  int n = 1;
  for (int i = 1; i <= kPermutationSize; ++i) {
    n *= i;
  }
  return n;
}

struct IntStruct {
  int value;
};

// computes next permutation in lexicographical order in place
// returns false if there is no next permutation
bool NextPermutation(array<int, kPermutationSize>& arr) {
  int j;
  for (j = arr.size() - 2; j >= 0; --j) {
    if (arr[j] < arr[j + 1]) {
      break;
    }
  }

  // last permutation i.e. [n, n - 1, ..., 3, 2, 1]
  if (j < 0) {
    return false;
  }

  for (int l = arr.size() - 1; l > -1; --l) {
    if (arr[j] < arr[l]) {
      std::swap(arr[j], arr[l]);
      std::reverse(arr.begin() + j + 1, arr.end());
      return true;
    }
  }
  return true;
}

int IntCompare(int* i, int* j, void* context) {
  if (*i < *j) {
    return -1;
  } else if (*i == *j) {
    return 0;
  } else {
    return 1;
  }
}
mjSORT(IntSort, int, IntCompare);
mjPARTIAL_SORT(IntSelect, int, IntCompare);

int IntStructCompare(const IntStruct* x, const IntStruct* y, void* context) {
  IntStruct* a = (IntStruct*)x;
  IntStruct* b = (IntStruct*)y;
  if (a->value < b->value) {
    return -1;
  } else if (a->value == b->value) {
    return 0;
  } else {
    return 1;
  }
}
mjSORT(IntStructSort, IntStruct, IntStructCompare);
mjPARTIAL_SORT(IntStructSelect, IntStruct, IntStructCompare);

TEST_F(EngineSortTest, Sort) {
  int total = 0;
  array<int, kPermutationSize> initial, buf;
  for (int i = 0; i < kPermutationSize; ++i) {
    initial[i] = i + 1;
  }
  array<int, kPermutationSize> arr = initial;
  do {
    ++total;
    array<int, kPermutationSize> sorted_arr = arr;
    IntSort(sorted_arr.data(), buf.data(), sorted_arr.size(), nullptr);
    EXPECT_EQ(sorted_arr, initial);
  } while (NextPermutation(arr));
  ASSERT_EQ(total, factorial());
}

TEST_F(EngineSortTest, LargeSort) {
  array<int, 2500> arr, buf;
  int n = 1;
  for (int i = arr.size() - 1; i >= 0; --i) {
    arr[i] = n++;
  }
  IntSort(arr.data(), buf.data(), arr.size(), nullptr);
  for (int i = 0; i < arr.size(); ++i) {
    EXPECT_EQ(arr[i], i + 1);
  }
}

TEST_F(EngineSortTest, SortStruct) {
  vector<IntStruct> y = {{1}, {3}, {2}};
  vector<IntStruct> buf(3);
  IntStructSort(y.data(), buf.data(), y.size(), nullptr);
  EXPECT_EQ(y[0].value, 1);
  EXPECT_EQ(y[1].value, 2);
  EXPECT_EQ(y[2].value, 3);
}

TEST_F(EngineSortTest, Select) {
  int total = 0;
  array<int, kPermutationSize> initial, buf;
  for (int i = 0; i < kPermutationSize; ++i) {
    initial[i] = i + 1;
  }
  array<int, kPermutationSize> arr = initial;
  do {
    ++total;
    for (int k = 1; k <= kPermutationSize; ++k) {
      array<int, kPermutationSize> select_arr = arr;
      IntSelect(select_arr.data(), buf.data(), select_arr.size(), k, nullptr);
      for (int i = 0; i < k; ++i) {
        EXPECT_EQ(select_arr[i], initial[i]);
      }
    }
  } while (NextPermutation(arr));
  ASSERT_EQ(total, factorial());
}

TEST_F(EngineSortTest, LargeSelect) {
  array<int, 2500> arr;
  array<int, 100> buf;
  int n = 1;
  for (int i = arr.size() - 1; i >= 0; --i) {
    arr[i] = n++;
  }
  IntSelect(arr.data(), buf.data(), arr.size(), buf.size(), nullptr);
  for (int i = 0; i < buf.size(); ++i) {
    EXPECT_EQ(arr[i], i + 1);
  }
}

TEST_F(EngineSortTest, SelectStruct) {
  vector<IntStruct> y = {{1}, {3}, {2}};
  vector<IntStruct> buf(3);
  const int k = 2;
  IntStructSelect(y.data(), buf.data(), y.size(), k, nullptr);
  EXPECT_EQ(y[0].value, 1);
  EXPECT_EQ(y[1].value, 2);
}

}  // namespace
}  // namespace mujoco
